function [rd, boardName] = hdlcoder_ref_design_customization
%HDLCODER_REF_DESIGN_CUSTOMIZATION Register Ultra96-V2 reference designs.

    rd = { ...
        'Ultra96V2.vivado_lpddr4_2025_2.plugin_rd' ...
    };

    boardName = 'Avnet Ultra96-V2';
end
