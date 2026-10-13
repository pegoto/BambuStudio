#ifndef slic3r_GLGizmoSupportFins_hpp_
#define slic3r_GLGizmoSupportFins_hpp_

// Support Fins gizmo: breakaway support fins (printfins.com technique, see
// src/libslic3r/SupportFins/SupportFins.hpp) generated under a tilted part's overhangs and
// stored as ordinary model-part volumes of the object. The gizmo shows the overhang overlay,
// the options panel (1:1 with upstream options.json), Generate / Clear (Auto mode) and
// click-two-points walls plus fin selection / removal (Draw mode).

#include "GLGizmoBase.hpp"
#include "slic3r/GUI/GLModel.hpp"
#include "libslic3r/ObjectID.hpp"
#include "admesh/stl.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r {

class ModelObject;
struct SupportFinOptions;
namespace SupportFins { struct Topology; }

namespace GUI {

class MeshRaycaster;
enum class SLAGizmoEventType : unsigned char;

class GLGizmoSupportFins : public GLGizmoBase
{
public:
    GLGizmoSupportFins(GLCanvas3D& parent, unsigned int sprite_id);
    ~GLGizmoSupportFins() override;

    void data_changed(bool is_serializing) override;
    bool gizmo_event(SLAGizmoEventType action, const Vec2d& mouse_position, bool shift_down, bool alt_down, bool control_down) override;
    std::string get_icon_filename(bool is_dark_mode) const override;

protected:
    bool on_init() override;
    std::string on_get_name() const override;
    bool on_is_activable() const override;
    bool on_is_selectable() const override;
    CommonGizmosDataID on_get_requirements() const override;
    void on_set_state() override;
    void on_render() override;
    void on_render_for_picking() override {}
    void on_render_input_window(float x, float y, float bottom_limit) override;

private:
    enum class Mode { Auto, Draw };

    // Panel values in the units the controls show (percent controls x100), see options.json.
    struct Panel
    {
        bool  material_auto     = true;  // follow the object's filament (PLA family -> pla, PETG family -> petg)
        int   material          = 0;     // 0 pla, 1 petg (used when !material_auto)
        float threshold         = 45.f;  // deg from the plate
        bool  tines             = true;
        float tine_density      = 0.f;   // %, 0..100
        int   pad_style         = 1;     // 0 off, 1 auto, 2 light, 3 sure
        bool  sway              = false;
        float sway_grip_from    = 0.f;   // mm
        float sway_tine_spacing = 6.f;   // mm
        float sway_reach        = 15.f;  // %, 5..50
        int   cutout            = 0;     // 0 none, 1 diamond, 2 triangle, 3 arch, 4 lattice
        float coverage          = 50.f;  // %, 0..100
        bool  plate_only        = false;
    };

    // What the overlay was computed for; a change of any of it means re-analysing.
    struct PoseKey
    {
        size_t              object_id = 0;
        int                 instance  = -1;
        std::vector<size_t> volume_ids;
        std::vector<const void*> meshes;
        std::vector<double> matrices;
        bool operator==(const PoseKey& o) const;
        bool operator!=(const PoseKey& o) const { return !(*this == o); }
    };

    struct VolumeRaycaster
    {
        ObjectID                        volume_id;
        const void*                     mesh = nullptr;
        bool                            is_fin = false;
        Transform3d                     world = Transform3d::Identity();
        std::shared_ptr<MeshRaycaster>  raycaster;
    };

    // The selected object and its active instance, re-fetched from the selection every time
    // (never cached across a snapshot / reload). Returns nullptr when there is none.
    ModelObject* selected_object(int& obj_idx, int& instance_idx) const;
    double       print_layer_height(const ModelObject& mo) const;
    // Engine material index (0 pla, 1 petg) of the filament the object prints with, -1 when that
    // filament is neither family. filament_type gets the preset's filament_type either way.
    int          filament_material(const ModelObject& mo, std::string& filament_type) const;
    // The material the engine gets: the filament's when material_auto (PLA when it is neither), else the panel's.
    int          effective_material(const ModelObject& mo) const;
    SupportFinOptions build_options(const ModelObject& mo) const;
    void         load_panel_from(const ModelObject& mo);

    // Actions.
    void generate();
    void clear();
    void remove_selected_fin();
    void add_draw_point(const Vec3d& world_pos);
    void refresh_after_model_change(int obj_idx);

    // Overlay / analysis.
    PoseKey make_pose_key(const ModelObject& mo, int instance_idx) const;
    void    update_analysis_if_needed();
    void    rebuild_topology(const ModelObject& mo, int instance_idx);
    void    reanalyze();
    void    render_overlay();

    // Raycasting.
    void update_raycasters(const ModelObject& mo, int instance_idx);
    void reset_raycasters();
    // Index into m_raycasters of the closest hit, -1 if none. world_hit gets the hit in world coords.
    int  raycast(const Vec2d& mouse_pos, Vec3d& world_hit);

    void render_tooltip_icon(float x, float y);

    Mode        m_mode = Mode::Auto;
    Panel       m_panel;
    bool        m_show_all = false;      // "Show all settings" (app_config support_fins_show_all)
    ObjectID    m_panel_object_id;   // object the panel values were loaded from

    // overhang analysis of the posed part (support_fins_posed_part)
    std::unique_ptr<SupportFins::Topology> m_topology;
    PoseKey               m_pose_key;
    float                 m_overlay_threshold  = 45.f;  // committed threshold (slider released)
    float                 m_analyzed_threshold = -1.f;  // threshold the overlay was computed for
    std::vector<uint8_t>  m_face_class;      // per posed face: 0 none, 1 red (fins), 2 amber (too small)
    indexed_triangle_set  m_posed_part;
    float                 m_part_height    = 0.f; // posed part: Z extent, mm
    float                 m_part_min_width = 0.f; // posed part: narrowest XY extent over 10-degree steps, mm
    bool                  m_overlay_dirty = true; // GL models need rebuilding
    GLModel               m_red_model;
    GLModel               m_amber_model;
    int                   m_red_faces = 0;
    int                   m_amber_faces = 0;
    bool                  m_has_fins = false;
    bool                  m_stale = false;
    bool                  m_stale_dirty = true;  // re-evaluate m_has_fins / m_stale on the next render

    // Draw mode / fin selection
    std::vector<Vec3d>    m_draw_points;      // world coords, 0 or 1 pending point
    ObjectID              m_selected_fin;     // ModelVolume id of the selected fin
    GLModel               m_selected_fin_model;
    ObjectID              m_selected_fin_model_id;
    std::vector<VolumeRaycaster> m_raycasters;
    bool                  m_wait_for_up_event = false;

    std::string           m_readout;
    bool                  m_readout_is_error = false;

    std::map<std::string, wxString> m_desc;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoSupportFins_hpp_
