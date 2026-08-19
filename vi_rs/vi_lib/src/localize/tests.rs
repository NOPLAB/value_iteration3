use super::*;
use crate::Quaternion;

fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
    PoseView { x, y, yaw_rad: yaw }
}

/// 外周を壁で囲い、対称性崩しの内部ブロックを 1 つ置いた占有格子 (@0.05 m)。
fn walled_grid(size: i32) -> OccupancyGrid {
    let mut g = OccupancyGrid {
        width: size,
        height: size,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: vec![0i8; (size * size) as usize],
    };
    for i in 0..size {
        for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
            g.data[(y * size + x) as usize] = 100;
        }
    }
    for y in 10..14 {
        for x in 40..44 {
            g.data[(y * size + x) as usize] = 100;
        }
    }
    g
}

fn wrap_rad(d: f64) -> f64 {
    use std::f64::consts::PI;
    (d + PI).rem_euclid(2.0 * PI) - PI
}

#[test]
fn external_localizer_passes_pose_through() {
    let mut l = ExternalLocalizer::default();
    assert!(l.pose().is_none());
    // predict/observe は no-op のまま落ちないこと。
    l.predict(0.3, 20.0, 0.1);
    l.observe(&LaserScan::default());
    l.set_pose(pose(1.0, 2.0, 0.5));
    let p = l.pose().expect("set_pose 後は返す");
    assert_eq!((p.x, p.y, p.yaw_rad), (1.0, 2.0, 0.5));
    assert_eq!(l.quality(), 1.0);
}

/// grid: ずらしたシードから合成スキャンで真値へ収束すること (correct の本体)。
#[test]
fn grid_localizer_tightens_onto_the_true_pose() {
    let g = walled_grid(60); // 3m×3m @0.05
    let truth = pose(1.2, 1.5, 0.5);
    let bc = BeliefConfig {
        half_m: 1.0,
        beam_step: 1,
        init_sigma_xy_m: 0.3,
        init_sigma_theta_deg: 20.0,
        ..BeliefConfig::default()
    };
    let mut loc = GridLocalizer::new(&g, 36, bc);
    assert!(loc.pose().is_none(), "シード前は None");

    // 真値から 0.14m / 11° ずらして手動シード。
    loc.set_pose(pose(1.3, 1.4, 0.3));
    let scan = cast_scan(&g, truth, 36, 5.0);
    for _ in 0..6 {
        loc.observe(&scan);
        loc.predict(0.0, 0.0, 0.1); // 静止でも動作ノイズのぼかしは回る
    }

    let m = loc.pose().expect("収束後の平均");
    assert!(
        (m.x - truth.x).abs() < 0.1 && (m.y - truth.y).abs() < 0.1,
        "mean ({:.3}, {:.3}) が真値 ({:.3}, {:.3}) から遠い",
        m.x, m.y, truth.x, truth.y
    );
    assert!(
        wrap_rad(m.yaw_rad - truth.yaw_rad).abs() < 0.2,
        "yaw {:.3} が真値 {:.3} から遠い",
        m.yaw_rad, truth.yaw_rad
    );
    assert!(loc.quality() > 0.5, "観測一致度が低すぎる: {}", loc.quality());
}

/// grid: predict が指令どおり平均を進め、回すこと (動作モデル = 自分の cmd_vel)。
#[test]
fn grid_localizer_predict_advances_the_mean_along_the_heading() {
    let g = walled_grid(60);
    let mut loc =
        GridLocalizer::new(&g, 36, BeliefConfig { half_m: 1.0, ..BeliefConfig::default() });
    loc.set_pose(pose(1.5, 1.5, 0.0));

    // 前進 0.3 m/s × 1.0 s。
    for _ in 0..10 {
        loc.predict(0.3, 0.0, 0.1);
    }
    let m = loc.pose().expect("平均");
    assert!((m.x - 1.8).abs() < 0.08, "x = {:.3} (期待 1.8 付近)", m.x);
    assert!((m.y - 1.5).abs() < 0.05, "y = {:.3} (期待 1.5 のまま)", m.y);

    // その場旋回 90 deg/s × 1.0 s。
    for _ in 0..10 {
        loc.predict(0.0, 90.0, 0.1);
    }
    let m = loc.pose().expect("平均");
    assert!(
        wrap_rad(m.yaw_rad - std::f64::consts::FRAC_PI_2).abs() < 0.2,
        "yaw = {:.3} (期待 π/2 付近)",
        m.yaw_rad
    );
}

/// 10m×10m、非対称な内部構造つきの占有格子 (@0.05 m)。ブロック配置が
/// 場所ごとに違うスキャン特徴を作るので、大域再定位が一意に解ける。
fn tenm_grid() -> OccupancyGrid {
    let size = 200;
    let mut g = OccupancyGrid {
        width: size,
        height: size,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: vec![0i8; (size * size) as usize],
    };
    for i in 0..size {
        for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
            g.data[(y * size + x) as usize] = 100;
        }
    }
    for (x0, x1, y0, y1) in
        [(20, 30, 20, 30), (140, 170, 40, 48), (60, 66, 150, 180), (118, 126, 118, 126)]
    {
        for y in y0..y1 {
            for x in x0..x1 {
                g.data[(y * size + x) as usize] = 100;
            }
        }
    }
    g
}

/// adaptive: 整合した観測なら L0 に留まり、grid と同じ追跡をすること。
#[test]
fn adaptive_localizer_tracks_at_level0_like_grid() {
    let g = walled_grid(60);
    let truth = pose(1.2, 1.5, 0.5);
    let bc = BeliefConfig {
        half_m: 1.0,
        beam_step: 1,
        init_sigma_xy_m: 0.3,
        init_sigma_theta_deg: 20.0,
        ..BeliefConfig::default()
    };
    let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
    assert!(loc.pose().is_none(), "シード前は None");
    loc.set_pose(pose(1.3, 1.4, 0.3));
    let scan = cast_scan(&g, truth, 36, 5.0);
    for _ in 0..6 {
        loc.observe(&scan);
        loc.predict(0.0, 0.0, 0.1);
    }
    assert_eq!(loc.level(), 0, "整合した観測で拡張しないこと");
    let m = loc.pose().expect("収束後の平均");
    assert!(
        (m.x - truth.x).abs() < 0.1 && (m.y - truth.y).abs() < 0.1,
        "mean ({:.3}, {:.3}) が真値 ({:.3}, {:.3}) から遠い",
        m.x, m.y, truth.x, truth.y
    );
    assert!(wrap_rad(m.yaw_rad - truth.yaw_rad).abs() < 0.2);
}

/// adaptive: 誘拐 (L1 窓の外へ瞬間移動) から expansion resetting で復帰する
/// こと。ロスト検出 (pose None) → 全域レベルまで拡張 → 再定位、の全カスケード。
#[test]
fn adaptive_localizer_recovers_from_kidnap() {
    let g = tenm_grid();
    let bc = BeliefConfig { half_m: 1.0, beam_step: 4, ..BeliefConfig::default() };
    let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
    assert!(loc.num_levels() >= 3);

    let a = pose(2.5, 2.0, 0.4);
    loc.set_pose(a);
    let scan_a = cast_scan(&g, a, 180, 12.0);
    for _ in 0..5 {
        loc.observe(&scan_a);
    }
    assert_eq!(loc.level(), 0);
    assert!(loc.pose().is_some());

    // 誘拐: 約 8m 離れた場所へ (L1 窓 8m の外 → L2 全域まで上がるはず)。
    let b = pose(8.0, 8.0, 2.0);
    let scan_b = cast_scan(&g, b, 180, 12.0);
    let mut max_level = 0;
    let mut lost = false;
    let mut recovered = None;
    for i in 0..150 {
        loc.observe(&scan_b);
        max_level = max_level.max(loc.level());
        match loc.pose() {
            None => lost = true,
            Some(p) => {
                if lost
                    && (p.x - b.x).abs() < 0.4
                    && (p.y - b.y).abs() < 0.4
                    && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                {
                    recovered = Some(i);
                    break;
                }
            }
        }
    }
    assert!(lost, "誘拐でロスト (pose None) を検出すること");
    assert!(max_level >= 2, "全域レベルまで拡張すること (max_level={max_level})");
    assert!(
        recovered.is_some(),
        "150 スキャン以内に再定位すること (max_level={max_level}, q={:.3})",
        loc.quality()
    );
}

/// adaptive: 未シードでも最初のスキャンから大域初期化で立ち上がること。
#[test]
fn adaptive_localizer_global_init_without_seed() {
    let g = tenm_grid();
    let bc = BeliefConfig { half_m: 1.0, beam_step: 4, ..BeliefConfig::default() };
    let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
    assert!(loc.pose().is_none());
    let truth = pose(8.0, 8.0, 2.0);
    let scan = cast_scan(&g, truth, 180, 12.0);
    let mut ok = None;
    for i in 0..150 {
        loc.observe(&scan);
        if let Some(p) = loc.pose() {
            if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                ok = Some(i);
                break;
            }
        }
    }
    assert!(ok.is_some(), "未シードでも大域初期化で再定位すること");
}

/// 推定が壁・未知の中に落ちないこと (free マスク + free スナップ)。
/// ブロックのど真ん中へシードすると、マスクが質量を周囲の free へ追い出し、
/// リング状に残った belief の平均はブロック内 (穴) に戻る — pose() は free 上の
/// mode へ吸着して返すはず。マスクかスナップのどちらが欠けても落ちる。
#[test]
fn estimate_never_lands_in_occupied_space() {
    let g = walled_grid(80);
    let free_at = |p: PoseView| {
        let ix = ((p.x - g.origin_x) / g.resolution).floor() as i32;
        let iy = ((p.y - g.origin_y) / g.resolution).floor() as i32;
        (0..g.width).contains(&ix)
            && (0..g.height).contains(&iy)
            && g.data[(iy * g.width + ix) as usize] == 0
    };
    // walled_grid の内部ブロック (x40..44, y10..14) の中心。
    let block_center = pose(42.0 * 0.05, 12.0 * 0.05, 0.0);

    let mut gl = GridLocalizer::new(&g, 36, BeliefConfig::default());
    gl.set_pose(block_center);
    let p = gl.pose().expect("マスク後も free 側に質量が残ること");
    assert!(free_at(p), "grid: 推定 ({:.2}, {:.2}) が free でない", p.x, p.y);

    let mut al = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
    al.set_pose(block_center);
    let p = al.pose().expect("マスク後も free 側に質量が残ること");
    assert!(free_at(p), "adaptive: 推定 ({:.2}, {:.2}) が free でない", p.x, p.y);
}

/// viterbi (min-plus 全域レベル): 誘拐から復帰すること — adaptive の
/// sum-product L2 を再帰 MAP に替えても同じ復帰性が保たれる回帰。
#[test]
fn viterbi_localizer_recovers_from_kidnap() {
    let g = tenm_grid();
    let bc =
        BeliefConfig { half_m: 1.0, beam_step: 4, viterbi: true, ..BeliefConfig::default() };
    let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
    assert_eq!(loc.name(), "viterbi");

    let a = pose(2.5, 2.0, 0.4);
    loc.set_pose(a);
    let scan_a = cast_scan(&g, a, 180, 12.0);
    for _ in 0..5 {
        loc.observe(&scan_a);
    }
    assert_eq!(loc.level(), 0);

    let b = pose(8.0, 8.0, 2.0);
    let scan_b = cast_scan(&g, b, 180, 12.0);
    let (mut lost, mut recovered) = (false, None);
    for i in 0..150 {
        loc.observe(&scan_b);
        match loc.pose() {
            None => lost = true,
            Some(p) => {
                if lost
                    && (p.x - b.x).abs() < 0.4
                    && (p.y - b.y).abs() < 0.4
                    && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                {
                    recovered = Some(i);
                    break;
                }
            }
        }
    }
    assert!(lost, "誘拐でロスト (pose None) を検出すること");
    assert!(
        recovered.is_some(),
        "150 スキャン以内に再定位すること (level={}, q={:.3})",
        loc.level(),
        loc.quality()
    );
}

/// viterbi: 未シードの大域初期化 (enter_uniform → δ 一様) でも立ち上がること。
#[test]
fn viterbi_localizer_global_init_without_seed() {
    let g = tenm_grid();
    let bc =
        BeliefConfig { half_m: 1.0, beam_step: 4, viterbi: true, ..BeliefConfig::default() };
    let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
    assert!(loc.pose().is_none());
    let truth = pose(8.0, 8.0, 2.0);
    let scan = cast_scan(&g, truth, 180, 12.0);
    let mut ok = None;
    for i in 0..150 {
        loc.observe(&scan);
        // ロスト中の predict は移動量の記録だけ (O(1)) — 静止でも落ちないこと。
        loc.predict(0.0, 0.0, 0.1);
        if let Some(p) = loc.pose() {
            if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                ok = Some(i);
                break;
            }
        }
    }
    assert!(ok.is_some(), "大域初期化から再定位できない (level={})", loc.level());
}

/// reloc_targets: 対称な 2 仮説から「地図が仮説間で違って見える方向」への
/// 変位を選ぶこと (能動的再定位の行き先)。北側の一方にだけ障害物クラスタが
/// ある地図で、両仮説とも北向きの行き先が返るはず。
#[test]
fn reloc_targets_point_toward_disambiguating_terrain() {
    // 20m×10m @0.1、開けた空間 + 仮説 A の北にだけ障害物クラスタ。
    let (w, h) = (200, 100);
    let mut g = OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.1,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: vec![0i8; (w * h) as usize],
    };
    for y in 90..96 {
        for x in 40..70 {
            g.data[(y * w + x) as usize] = 100;
        }
    }
    let mut loc = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
    assert!(loc.reloc_targets().is_empty(), "未初期化は空");

    // ロスト状態を直接組む: 全域レベルで (5.2, 5.2) と (15.2, 5.2) の 2 仮説。
    loc.force_bimodal_global((5.2, 5.2, 0.6), (15.2, 5.2, 0.4));

    let t = loc.reloc_targets();
    assert_eq!(t.len(), 2, "仮説ごとに 1 点");
    for &(x, y) in &t {
        assert!(y > 6.0, "行き先 ({x:.1}, {y:.1}) が判別地形 (北) を向いていない");
    }
    // 同じロボット系変位 δ (両仮説とも yaw ビン 0) — 世界系でも同じずれ。
    assert!(
        ((t[1].0 - t[0].0) - 10.0).abs() < 0.5,
        "2 つの行き先は同じ δ で結ばれるはず: {t:?}"
    );
}

/// top_cells: シード直後の最大重み仮説がシード姿勢のセルで、重みが降順なこと
/// (QMDP の仮説集合の契約)。grid / adaptive 両方。
#[test]
fn top_cells_returns_descending_hypotheses_near_the_seed() {
    let g = walled_grid(60);
    let seed = pose(1.5, 1.5, 0.0);
    let check = |cells: Vec<(PoseView, f64)>, name: &str| {
        assert!(!cells.is_empty(), "{name}: 仮説が空");
        assert!(cells.len() <= 8, "{name}: k を超過");
        for w in cells.windows(2) {
            assert!(w[0].1 >= w[1].1, "{name}: 重みが降順でない");
        }
        let top = cells[0].0;
        assert!(
            (top.x - seed.x).abs() < 0.1 && (top.y - seed.y).abs() < 0.1,
            "{name}: 最大重み仮説 ({:.2}, {:.2}) がシードから遠い",
            top.x, top.y
        );
    };
    let mut gl =
        GridLocalizer::new(&g, 36, BeliefConfig { half_m: 1.0, ..BeliefConfig::default() });
    assert!(gl.top_cells(8).is_empty(), "シード前は空");
    gl.set_pose(seed);
    check(gl.top_cells(8), "grid");

    let mut al = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
    al.set_pose(seed);
    check(al.top_cells(8), "adaptive");
}

/// 窓つき 2 実装の可視化グリッド: シード前は None、シード後は窓の範囲を
/// 覆う格子が出て、ピークがシード位置に立つ。
#[test]
fn belief_grid_covers_the_window_with_its_peak_at_the_seed() {
    let g = walled_grid(60);
    let seed = pose(1.5, 1.5, 0.0);
    let check = |vg: OccupancyGrid, name: &str| {
        let (i, &v) = vg.data.iter().enumerate().max_by_key(|(_, &v)| v).unwrap();
        let (px, py) = (
            vg.origin_x + (i as i32 % vg.width) as f64 * vg.resolution,
            vg.origin_y + (i as i32 / vg.width) as f64 * vg.resolution,
        );
        assert!(v <= 98, "{name}: スケール上限を超えた: {v}");
        assert!(
            (px - seed.x).abs() < 0.2 && (py - seed.y).abs() < 0.2,
            "{name}: ピーク ({px:.2}, {py:.2}) がシードから遠い"
        );
    };
    let mut gl =
        GridLocalizer::new(&g, 36, BeliefConfig { half_m: 1.0, ..BeliefConfig::default() });
    assert!(gl.belief_grid().is_none(), "シード前は None");
    gl.set_pose(seed);
    check(gl.belief_grid().expect("grid"), "grid");

    let mut al = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
    al.set_pose(seed);
    check(al.belief_grid().expect("adaptive"), "adaptive");
}
