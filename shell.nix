{ pkgs ? import <nixpkgs> { } }:

pkgs.mkShell {
  name = "value-iteration3";

  packages = with pkgs; [
    cargo
    rustc
    rustfmt
    clippy
    rust-analyzer
    pkg-config
    gnumake
    git
    docker-client
  ];

  RUST_SRC_PATH = "${pkgs.rustPlatform.rustLibSrc}";
}
