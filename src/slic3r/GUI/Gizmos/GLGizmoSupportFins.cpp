// Include GLGizmoBase.hpp before I18N.hpp as it includes some libigl code, which overrides our localization "L" macro.
#include "GLGizmoSupportFins.hpp"
#include <GL/glew.h>
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/MeshUtils.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/format.hpp"
#include "libslic3r/SupportFins/SupportFins.hpp"
#include "libslic3r/SupportFins/Engine/Overhangs.hpp"

#include <wx/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

namespace Slic3r { namespace GUI {

// Overlay colours (upstream site: overFaces red, smallFaces amber).
static const ColorRGBA OVERHANG_COLOR     = {0.9f, 0.2f, 0.2f, 0.6f};
static const ColorRGBA TOO_SMALL_COLOR    = {0.95f, 0.7f, 0.2f, 0.6f};
static const ColorRGBA SELECTED_FIN_COLOR = {0.95f, 0.7f, 0.2f, 0.5f};
static const ColorRGBA DRAW_POINT_COLOR   = {0.0f, 0.68f, 0.26f, 1.0f};
static const ImVec4    HINT_TEXT_COLOR    = {0.42f, 0.42f, 0.42f, 1.0f};

// options.json choices, in schema order.
static const char *MATERIAL_VALUES[] = {"pla", "petg"};
static const char *PAD_VALUES[]      = {"off", "auto", "light", "sure"};
static const char *CUTOUT_VALUES[]   = {"none", "diamond", "triangle", "arch", "lattice"};

template<size_t N> static int choice_index(const char *(&values)[N], const std::string &value, int fallback)
{
    for (size_t i = 0; i < N; ++i)
        if (value == values[i])
            return int(i);
    return fallback;
}

// Round to the control's step and clamp to its range (options.json: "round to step for display").
static float snap_to_step(float v, float step, float v_min, float v_max)
{
    if (step > 0.f)
        v = std::round(v / step) * step;
    return std::clamp(v, v_min, v_max);
}

bool GLGizmoSupportFins::PoseKey::operator==(const PoseKey &o) const
{
    return object_id == o.object_id && instance == o.instance && volume_ids == o.volume_ids && meshes == o.meshes && matrices == o.matrices;
}

GLGizmoSupportFins::GLGizmoSupportFins(GLCanvas3D &parent, unsigned int sprite_id) : GLGizmoBase(parent, sprite_id) {}

GLGizmoSupportFins::~GLGizmoSupportFins() = default;

bool GLGizmoSupportFins::on_init()
{
    // No single-key shortcut: the free letters are taken by the other gizmos.
    m_desc["left_click_caption"] = _L("Left click");
    m_desc["left_click"]         = _L("Select a fin. In Draw mode: pick a wall end on the part");
    m_desc["delete_caption"]     = _L("Delete");
    m_desc["delete"]             = _L("Remove the selected fin");
    m_desc["esc_caption"]        = _L("Esc");
    m_desc["esc"]                = _L("Cancel the first point, or leave");
    return true;
}

std::string GLGizmoSupportFins::get_icon_filename(bool is_dark_mode) const
{
    return is_dark_mode ? "toolbar_supportfins_dark.svg" : "toolbar_supportfins.svg";
}

std::string GLGizmoSupportFins::on_get_name() const
{
    if (!on_is_activable() && m_state == EState::Off)
        return _u8L("Support Fins") + ":\n" + _u8L("Please select single object.");
    return _u8L("Support Fins");
}

bool GLGizmoSupportFins::on_is_selectable() const
{
    return wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptFFF;
}

bool GLGizmoSupportFins::on_is_activable() const
{
    return wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptFFF
        && m_parent.get_selection().is_single_full_instance();
}

CommonGizmosDataID GLGizmoSupportFins::on_get_requirements() const
{
    return CommonGizmosDataID(int(CommonGizmosDataID::SelectionInfo) | int(CommonGizmosDataID::InstancesHider));
}

ModelObject *GLGizmoSupportFins::selected_object(int &obj_idx, int &instance_idx) const
{
    const Selection &selection = m_parent.get_selection();
    obj_idx      = selection.get_object_idx();
    instance_idx = selection.get_instance_idx();
    const Model *model = selection.get_model();
    if (model == nullptr || obj_idx < 0 || obj_idx >= int(model->objects.size()))
        return nullptr;
    ModelObject *mo = model->objects[obj_idx];
    if (instance_idx < 0 || instance_idx >= int(mo->instances.size()))
        return nullptr;
    return mo;
}

double GLGizmoSupportFins::print_layer_height(const ModelObject &mo) const
{
    // options.json: layerHeight is hostSupplied = 'slicer' -> the slice layer height, object override first.
    const DynamicPrintConfig &obj_cfg = mo.config.get();
    if (const ConfigOption *opt = obj_cfg.option("layer_height"); opt != nullptr)
        return opt->getFloat();
    const DynamicPrintConfig &print_cfg = wxGetApp().preset_bundle->prints.get_edited_preset().config;
    if (print_cfg.has("layer_height"))
        return print_cfg.opt_float("layer_height");
    return SupportFinOptions().layer_height;
}

SupportFinOptions GLGizmoSupportFins::build_options(const ModelObject &mo) const
{
    SupportFinOptions o;
    o.material          = MATERIAL_VALUES[std::clamp(m_panel.material, 0, 1)];
    o.threshold         = m_panel.threshold;
    o.tines             = m_panel.tines;
    o.tine_density      = m_panel.tine_density / 100.;
    o.layer_height      = print_layer_height(mo);
    o.pad_style         = PAD_VALUES[std::clamp(m_panel.pad_style, 0, 3)];
    o.sway              = m_panel.sway;
    o.sway_grip_from    = m_panel.sway_grip_from;
    o.sway_tine_spacing = m_panel.sway_tine_spacing;
    o.sway_reach        = m_panel.sway_reach / 100.;
    o.cutout            = CUTOUT_VALUES[std::clamp(m_panel.cutout, 0, 4)];
    o.coverage          = m_panel.coverage / 100.;
    o.plate_only        = m_panel.plate_only;
    o.validate();
    return o;
}

void GLGizmoSupportFins::load_panel_from(const ModelObject &mo)
{
    m_panel = Panel();
    if (mo.support_fin_options.has_value()) {
        const SupportFinOptions &o = *mo.support_fin_options;
        m_panel.material          = choice_index(MATERIAL_VALUES, o.material, 0);
        m_panel.threshold         = float(o.threshold);
        m_panel.tines             = o.tines;
        m_panel.tine_density      = float(o.tine_density * 100.);
        m_panel.pad_style         = choice_index(PAD_VALUES, o.pad_style, 1);
        m_panel.sway              = o.sway;
        m_panel.sway_grip_from    = float(o.sway_grip_from);
        m_panel.sway_tine_spacing = float(o.sway_tine_spacing);
        m_panel.sway_reach        = float(o.sway_reach * 100.);
        m_panel.cutout            = choice_index(CUTOUT_VALUES, o.cutout, 0);
        m_panel.coverage          = float(o.coverage * 100.);
        m_panel.plate_only        = o.plate_only;
    }
    m_overlay_threshold = m_panel.threshold;
}

// ---------------------------------------------------------------------------------------------
// State / data
// ---------------------------------------------------------------------------------------------

void GLGizmoSupportFins::on_set_state()
{
    if (m_state == On) {
        // Force data_changed() to reload the panel from the object and the overlay to be recomputed.
        m_panel_object_id = ObjectID();
        m_pose_key        = PoseKey();
        m_topology.reset();
        m_stale_dirty     = true;
        m_readout.clear();
        m_readout_is_error = false;
    } else if (m_state == Off) {
        m_draw_points.clear();
        m_selected_fin          = ObjectID();
        m_selected_fin_model_id = ObjectID();
        m_wait_for_up_event     = false;
        reset_raycasters();
        // Free the analysis; the GL models are reset on the next render pass (needs the GL context).
        m_topology.reset();
        m_pose_key = PoseKey();
        m_posed_part.clear();
        m_face_class.clear();
        m_overlay_dirty = true;
    }
    GLGizmoBase::on_set_state();
}

void GLGizmoSupportFins::data_changed(bool is_serializing)
{
    if (m_c == nullptr || m_c->selection_info() == nullptr)
        return;
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    if (mo->id() != m_panel_object_id) {
        load_panel_from(*mo);
        m_panel_object_id = mo->id();
        m_draw_points.clear();
        m_selected_fin = ObjectID();
        m_readout.clear();
        m_readout_is_error = false;
    }

    // The volumes may have been replaced (undo/redo, reload): drop a selection that no longer exists.
    if (m_selected_fin.valid()) {
        bool found = false;
        for (const ModelVolume *mv : mo->volumes)
            if (mv->id() == m_selected_fin && mv->is_support_fin()) { found = true; break; }
        if (!found)
            m_selected_fin = ObjectID();
    }
    m_selected_fin_model_id = ObjectID(); // rebuild the highlight, its transform may have changed
    m_stale_dirty           = true;
    update_raycasters(*mo, inst_idx);
}

GLGizmoSupportFins::PoseKey GLGizmoSupportFins::make_pose_key(const ModelObject &mo, int instance_idx) const
{
    PoseKey key;
    key.object_id = mo.id().id;
    key.instance  = instance_idx;
    auto push_matrix = [&key](const Transform3d &m) {
        const double *d = m.data();
        key.matrices.insert(key.matrices.end(), d, d + 16);
    };
    push_matrix(mo.instances[instance_idx]->get_matrix());
    for (const ModelVolume *mv : mo.volumes) {
        if (!mv->is_model_part() || mv->is_support_fin())
            continue;
        key.volume_ids.push_back(mv->id().id);
        key.meshes.push_back(mv->mesh_ptr());
        push_matrix(mv->get_matrix());
    }
    return key;
}

void GLGizmoSupportFins::update_analysis_if_needed()
{
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    PoseKey key = make_pose_key(*mo, inst_idx);
    if (key != m_pose_key) {
        rebuild_topology(*mo, inst_idx);
        m_pose_key           = std::move(key);
        m_analyzed_threshold = -1.f;
        m_stale_dirty        = true;
        m_selected_fin_model_id = ObjectID();
    }
    if (m_analyzed_threshold != m_overlay_threshold)
        reanalyze();
    if (m_stale_dirty) {
        // support_fins_stale() hashes the posed part: only on a model / pose change, never per frame.
        m_has_fins    = mo->has_support_fins();
        m_stale       = m_has_fins && support_fins_stale(*mo, inst_idx);
        m_stale_dirty = false;
    }
}

void GLGizmoSupportFins::rebuild_topology(const ModelObject &mo, int instance_idx)
{
    m_topology.reset();
    m_face_class.clear();
    m_overlay_dirty = true;
    try {
        m_posed_part = support_fins_posed_part(mo, instance_idx);
        if (m_posed_part.indices.empty())
            return;
        SupportFins::Soup soup;
        soup.reserve(m_posed_part.indices.size() * 9);
        for (const stl_triangle_vertex_indices &f : m_posed_part.indices)
            for (int k = 0; k < 3; ++k) {
                const stl_vertex &v = m_posed_part.vertices[f[k]];
                soup.push_back(double(v.x()));
                soup.push_back(double(v.y()));
                soup.push_back(double(v.z()));
            }
        m_topology = std::make_unique<SupportFins::Topology>(SupportFins::buildTopology(std::move(soup)));
    } catch (const std::exception &) {
        m_topology.reset();
        m_posed_part.clear();
    }
}

void GLGizmoSupportFins::reanalyze()
{
    m_face_class.assign(m_posed_part.indices.size(), 0);
    m_red_faces = m_amber_faces = 0;
    if (m_topology) {
        try {
            SupportFins::AnalyzeResult res = SupportFins::analyze(*m_topology, double(m_overlay_threshold), SupportFins::IDENTITY3);
            const size_t n = std::min(m_face_class.size(), res.over.size());
            for (size_t f = 0; f < n; ++f) {
                if (!res.over[f])
                    continue;
                // kept = past the threshold and in a region big enough to fin (red); the rest of `over` is too small (amber).
                const bool kept = f < res.kept.size() && res.kept[f];
                m_face_class[f] = kept ? 1 : 2;
                ++(kept ? m_red_faces : m_amber_faces);
            }
        } catch (const std::exception &) {
            std::fill(m_face_class.begin(), m_face_class.end(), uint8_t(0));
            m_red_faces = m_amber_faces = 0;
        }
    }
    m_analyzed_threshold = m_overlay_threshold;
    m_overlay_dirty      = true;
}

// ---------------------------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------------------------

void GLGizmoSupportFins::on_render()
{
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);

    // If the selection no longer matches the object the gizmo works on, ask GLCanvas3D to turn us off.
    if (m_state == On && (mo == nullptr || m_c == nullptr || m_c->selection_info() == nullptr || mo != m_c->selection_info()->model_object() ||
                          m_c->selection_info()->get_active_instance() != inst_idx)) {
        m_parent.post_event(SimpleEvent(EVT_GLCANVAS_RESETGIZMOS));
        return;
    }

    update_analysis_if_needed();
    render_overlay();
}

void GLGizmoSupportFins::render_overlay()
{
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    if (m_overlay_dirty) {
        indexed_triangle_set red, amber;
        const size_t n = std::min(m_face_class.size(), m_posed_part.indices.size());
        for (size_t f = 0; f < n; ++f) {
            if (m_face_class[f] == 0)
                continue;
            indexed_triangle_set &dst  = m_face_class[f] == 1 ? red : amber;
            const int             base = int(dst.vertices.size());
            for (int k = 0; k < 3; ++k)
                dst.vertices.push_back(m_posed_part.vertices[m_posed_part.indices[f][k]]);
            dst.indices.emplace_back(base, base + 1, base + 2);
        }
        m_red_model.reset();
        m_amber_model.reset();
        if (!red.indices.empty()) {
            m_red_model.init_from(red);
            m_red_model.set_color(OVERHANG_COLOR);
        }
        if (!amber.indices.empty()) {
            m_amber_model.init_from(amber);
            m_amber_model.set_color(TOO_SMALL_COLOR);
        }
        m_overlay_dirty = false;
    }

    // Also drop the highlight whenever nothing is selected: the deselect paths (remove, generate,
    // undo, closing the gizmo) reset both ids to invalid, which alone would leave the old mesh drawn.
    if (m_selected_fin_model_id != m_selected_fin || (!m_selected_fin.valid() && m_selected_fin_model.is_initialized())) {
        m_selected_fin_model.reset();
        if (m_selected_fin.valid()) {
            const Transform3d inst_m = mo->instances[inst_idx]->get_matrix();
            for (const ModelVolume *mv : mo->volumes) {
                if (mv->id() != m_selected_fin)
                    continue;
                indexed_triangle_set its = mv->mesh().its;
                const Transform3d    m   = inst_m * mv->get_matrix();
                for (stl_vertex &v : its.vertices)
                    v = (m * v.cast<double>()).cast<float>();
                if (!its.indices.empty()) {
                    m_selected_fin_model.init_from(its);
                    m_selected_fin_model.set_color(SELECTED_FIN_COLOR);
                }
                break;
            }
        }
        m_selected_fin_model_id = m_selected_fin;
    }

    const bool has_overlay = m_red_model.is_initialized() || m_amber_model.is_initialized() || m_selected_fin_model.is_initialized() ||
                             !m_draw_points.empty();
    if (!has_overlay)
        return;

    const auto &shader = wxGetApp().get_shader("flat");
    if (shader == nullptr)
        return;

    const GLboolean cull_enabled = ::glIsEnabled(GL_CULL_FACE);
    glsafe(::glEnable(GL_DEPTH_TEST));
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glDisable(GL_CULL_FACE));
    // Draw the tint over the model's own (coplanar) faces without z-fighting.
    glsafe(::glEnable(GL_POLYGON_OFFSET_FILL));
    glsafe(::glPolygonOffset(-1.f, -1.f));
    glsafe(::glDepthMask(GL_FALSE));

    wxGetApp().bind_shader(shader);
    const Camera &camera = wxGetApp().plater()->get_camera();
    shader->set_uniform("projection_matrix", camera.get_projection_matrix());
    // The overlay meshes are already in world coordinates.
    shader->set_uniform("view_model_matrix", camera.get_view_matrix());
    if (m_red_model.is_initialized())
        m_red_model.render_geometry();
    if (m_amber_model.is_initialized())
        m_amber_model.render_geometry();
    if (m_selected_fin_model.is_initialized())
        m_selected_fin_model.render_geometry();

    // The first point of a wall being drawn.
    for (const Vec3d &p : m_draw_points) {
        const double radius = 0.8;
        shader->set_uniform("view_model_matrix",
                            camera.get_view_matrix() * Geometry::assemble_transform(p, Vec3d::Zero(), Vec3d(radius, radius, radius)));
        m_sphere.set_color(DRAW_POINT_COLOR);
        m_sphere.render_geometry();
    }
    wxGetApp().unbind_shader();

    glsafe(::glDepthMask(GL_TRUE));
    glsafe(::glDisable(GL_POLYGON_OFFSET_FILL));
    if (cull_enabled)
        glsafe(::glEnable(GL_CULL_FACE));
    glsafe(::glDisable(GL_BLEND));
}

// ---------------------------------------------------------------------------------------------
// Raycasting and mouse
// ---------------------------------------------------------------------------------------------

void GLGizmoSupportFins::reset_raycasters() { m_raycasters.clear(); }

void GLGizmoSupportFins::update_raycasters(const ModelObject &mo, int instance_idx)
{
    // One raycaster per model-part volume (part and fins). They reference the volume meshes,
    // so they are rebuilt whenever a volume or its mesh changes (compared by id / pointer only).
    std::vector<const ModelVolume *> parts;
    for (const ModelVolume *mv : mo.volumes)
        if (mv->is_model_part())
            parts.push_back(mv);

    bool same = parts.size() == m_raycasters.size();
    for (size_t i = 0; same && i < parts.size(); ++i)
        same = m_raycasters[i].volume_id == parts[i]->id() && m_raycasters[i].mesh == parts[i]->mesh_ptr();
    if (!same) {
        m_raycasters.clear();
        m_raycasters.reserve(parts.size());
        for (const ModelVolume *mv : parts) {
            VolumeRaycaster rc;
            rc.volume_id = mv->id();
            rc.mesh      = mv->mesh_ptr();
            rc.raycaster = std::make_shared<MeshRaycaster>(mv->mesh());
            m_raycasters.emplace_back(std::move(rc));
        }
    }
    const Transform3d &inst_m = mo.instances[instance_idx]->get_matrix();
    for (size_t i = 0; i < parts.size(); ++i) {
        m_raycasters[i].is_fin = parts[i]->is_support_fin();
        m_raycasters[i].world  = inst_m * parts[i]->get_matrix();
    }
}

int GLGizmoSupportFins::raycast(const Vec2d &mouse_pos, Vec3d &world_hit)
{
    const Camera &camera  = wxGetApp().plater()->get_camera();
    const Vec3d   eye     = camera.get_position();
    int           best    = -1;
    double        best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < m_raycasters.size(); ++i) {
        const VolumeRaycaster &rc = m_raycasters[i];
        Vec3f                  hit, normal;
        if (rc.raycaster && rc.raycaster->unproject_on_mesh(mouse_pos, rc.world, camera, hit, normal)) {
            const Vec3d  w  = rc.world * hit.cast<double>();
            const double d2 = (w - eye).squaredNorm();
            if (d2 < best_d2) {
                best_d2   = d2;
                best      = int(i);
                world_hit = w;
            }
        }
    }
    return best;
}

// Following function is called from GLCanvas3D to inform the gizmo about a mouse/keyboard event.
// Returns true if the gizmo used the event.
bool GLGizmoSupportFins::gizmo_event(SLAGizmoEventType action, const Vec2d &mouse_position, bool shift_down, bool alt_down, bool control_down)
{
    if (action == SLAGizmoEventType::LeftDown) {
        if (shift_down || alt_down)
            return false;
        int          obj_idx = -1, inst_idx = -1;
        ModelObject *mo = selected_object(obj_idx, inst_idx);
        if (mo == nullptr)
            return false;
        update_raycasters(*mo, inst_idx);
        Vec3d     hit = Vec3d::Zero();
        const int idx = raycast(mouse_position, hit);
        if (idx < 0)
            return false;
        if (m_raycasters[idx].is_fin) {
            // Clicking a fin selects it (amber), Delete / "Remove selected" removes it.
            m_selected_fin      = m_raycasters[idx].volume_id;
            m_wait_for_up_event = true;
            m_parent.set_as_dirty();
            return true;
        }
        m_selected_fin = ObjectID();
        if (m_mode == Mode::Draw) {
            m_wait_for_up_event = true;
            add_draw_point(hit);
            m_parent.set_as_dirty();
            return true;
        }
        m_parent.set_as_dirty();
        return false;
    }

    if (action == SLAGizmoEventType::LeftUp) {
        if (m_wait_for_up_event) {
            m_wait_for_up_event = false;
            return true;
        }
        return false;
    }

    if (action == SLAGizmoEventType::Dragging)
        // A point / fin has been picked and the button is not released yet:
        // this prevents GLCanvas from starting a scene rotation.
        return m_wait_for_up_event;

    if (action == SLAGizmoEventType::Delete) {
        // Always consumed, so Delete never falls through to deleting the whole object.
        remove_selected_fin();
        return true;
    }

    if (action == SLAGizmoEventType::DiscardChanges) {
        // Esc: drop a pending first point instead of closing the gizmo.
        if (!m_draw_points.empty()) {
            m_draw_points.clear();
            m_readout.clear();
            m_parent.set_as_dirty();
            return true;
        }
        return false;
    }

    return false;
}

// ---------------------------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------------------------

void GLGizmoSupportFins::refresh_after_model_change(int obj_idx)
{
    ObjectList *obj_list = wxGetApp().obj_list();
    const ModelObject *mo = wxGetApp().model().objects[obj_idx];
    if (mo->volumes.size() > 1) {
        // Sorts the volumes, rebuilds the object's volume items and info items and calls changed_object()
        // (ensure_on_bed + GLCanvas3D::reload_scene + schedule_background_process).
        obj_list->reorder_volumes_and_get_selection(obj_idx);
    } else {
        // reorder_volumes_and_get_selection() does nothing for a single volume: drop the stale fin items by hand.
        obj_list->update_info_items(obj_idx);
        obj_list->add_volumes_to_object_in_list(obj_idx);
        obj_list->changed_object(obj_idx);
    }
    wxGetApp().plater()->set_plater_dirty(true);

    m_selected_fin_model_id = ObjectID();
    m_stale_dirty           = true;
    m_parent.set_as_dirty();
}

void GLGizmoSupportFins::generate()
{
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    const SupportFinOptions options = build_options(*mo);
    m_draw_points.clear();
    m_selected_fin = ObjectID();
    // The raycasters reference the fin meshes apply_support_fins() is about to delete.
    reset_raycasters();

    SupportFinStats stats;
    {
        wxBusyCursor         wait;
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Generate support fins"));
        try {
            stats              = apply_support_fins(*mo, options, inst_idx);
            m_readout          = support_fins_report(stats);
            m_readout_is_error = false;
        } catch (const std::invalid_argument &e) {
            m_readout          = e.what();
            m_readout_is_error = true;
        } catch (const std::exception &e) {
            m_readout          = _u8L("Generating support fins failed") + ": " + e.what();
            m_readout_is_error = true;
        }
    }
    // Refresh even after an error: the engine may have removed the old fins before it threw.
    refresh_after_model_change(obj_idx);
}

void GLGizmoSupportFins::clear()
{
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr || (!mo->has_support_fins() && !mo->support_fin_options.has_value()))
        return;

    m_draw_points.clear();
    m_selected_fin = ObjectID();
    reset_raycasters();

    int removed = 0;
    {
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Clear support fins"));
        removed = clear_support_fins(*mo);
    }
    m_readout          = Slic3r::format(_u8L("Removed %1% support fin parts."), removed);
    m_readout_is_error = false;
    refresh_after_model_change(obj_idx);
}

void GLGizmoSupportFins::remove_selected_fin()
{
    if (!m_selected_fin.valid())
        return;
    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    int vol_idx = -1;
    for (size_t i = 0; i < mo->volumes.size(); ++i)
        if (mo->volumes[i]->id() == m_selected_fin && mo->volumes[i]->is_support_fin()) {
            vol_idx = int(i);
            break;
        }
    m_selected_fin = ObjectID();
    if (vol_idx < 0)
        return;

    reset_raycasters();
    {
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Remove support fin"));
        // The object list's delete path keeps the list in sync; it reloads the scene (changed_object).
        std::vector<ItemForDelete> items{ItemForDelete(itVolume, obj_idx, vol_idx)};
        wxGetApp().obj_list()->delete_from_model_and_list(items);
    }
    wxGetApp().plater()->set_plater_dirty(true);
    m_readout.clear();
    m_readout_is_error      = false;
    m_selected_fin_model_id = ObjectID();
    m_stale_dirty           = true;
    m_parent.set_as_dirty();
}

void GLGizmoSupportFins::add_draw_point(const Vec3d &world_pos)
{
    m_draw_points.push_back(world_pos);
    if (m_draw_points.size() < 2) {
        m_readout          = _u8L("Now click the second point.");
        m_readout_is_error = false;
        return;
    }

    const Vec3d a = m_draw_points[0];
    const Vec3d b = m_draw_points[1];
    m_draw_points.clear();

    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;
    const SupportFinOptions options = build_options(*mo);

    // Ask the engine first, so a refused wall does not leave an empty undo step behind.
    update_analysis_if_needed();
    std::string reason;
    bool        ok = false;
    try {
        ok = draw_support_fin_wall(m_posed_part, a, b, options, reason).has_value();
    } catch (const std::exception &e) {
        reason = e.what();
    }
    if (ok) {
        reset_raycasters();
        Plater::TakeSnapshot snapshot(wxGetApp().plater(), _u8L("Draw support fin"));
        try {
            ok = add_drawn_support_fin(*mo, a, b, options, reason, inst_idx);
        } catch (const std::exception &e) {
            ok     = false;
            reason = e.what();
        }
    }
    if (!ok) {
        m_readout          = reason.empty() ? _u8L("No wall can stand between these two points.") : reason;
        m_readout_is_error = true;
        return;
    }
    m_readout          = _u8L("Wall added.");
    m_readout_is_error = false;
    refresh_after_model_change(obj_idx);
}

// ---------------------------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------------------------

void GLGizmoSupportFins::render_tooltip_icon(float x, float y)
{
    std::array<std::string, 3> info_array  = {"left_click", "delete", "esc"};
    float                      caption_max = 0.f;
    for (const auto &t : info_array)
        caption_max = std::max(caption_max, m_imgui->calc_text_size(m_desc[t + "_caption"]).x);

    ImTextureID normal_id = m_parent.get_gizmos_manager().get_icon_texture_id(GLGizmosManager::MENU_ICON_NAME::IC_TOOLBAR_TOOLTIP);
    ImTextureID hover_id  = m_parent.get_gizmos_manager().get_icon_texture_id(GLGizmosManager::MENU_ICON_NAME::IC_TOOLBAR_TOOLTIP_HOVER);

    caption_max += m_imgui->calc_text_size(": ").x + 35.f;

    float  font_size   = ImGui::GetFontSize();
    ImVec2 button_size = ImVec2(font_size * 1.8, font_size * 1.3);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {0, ImGui::GetStyle().FramePadding.y});
    ImGui::ImageButton3(normal_id, hover_id, button_size);

    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip2(ImVec2(x, y));
        auto draw_text_with_caption = [this, &caption_max](const wxString &caption, const wxString &text) {
            m_imgui->text_colored(ImGuiWrapper::COL_ACTIVE, caption);
            ImGui::SameLine(caption_max);
            m_imgui->text_colored(ImGuiWrapper::COL_WINDOW_BG, text);
        };
        for (const auto &t : info_array)
            draw_text_with_caption(m_desc.at(t + "_caption") + ": ", m_desc.at(t));
        ImGui::EndTooltip();
    }
    ImGui::PopStyleVar(2);
}

void GLGizmoSupportFins::on_render_input_window(float x, float y, float bottom_limit)
{
    static float last_y = 0.0f;
    static float last_h = 0.0f;

    int          obj_idx = -1, inst_idx = -1;
    ModelObject *mo = selected_object(obj_idx, inst_idx);
    if (mo == nullptr)
        return;

    const float win_h = ImGui::GetWindowHeight();
    y                 = std::min(y, bottom_limit - win_h);
    GizmoImguiSetNextWIndowPos(x, y, ImGuiCond_Always, 0.0f, 0.0f);

    const float currt_scale = m_parent.get_scale();
    ImGuiWrapper::push_toolbar_style(currt_scale);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0 * currt_scale, 5.0 * currt_scale));
    GizmoImguiBegin(get_name(), ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                    ImGuiWindowFlags_NoTitleBar);

    // adjust window position to avoid overlap the view toolbar
    if (last_h != win_h || last_y != y) {
        // ask canvas for another frame to render the window in the correct position
        m_imgui->set_requires_extra_frame();
        if (last_h != win_h) last_h = win_h;
        if (last_y != y) last_y = y;
    }

    // Labels / hints / tooltips: plugins/shared/engine/options.json (the site's wording).
    const wxString lbl_mode        = _L("Mode");
    const wxString lbl_material    = _L("Material");
    const wxString lbl_threshold   = _L("Overhang");
    const wxString lbl_tines       = _L("Tines");
    const wxString lbl_tine_grip   = _L("Tine grip");
    const wxString lbl_pad         = _L("Bed pad");
    const wxString lbl_sway        = _L("Sway braces");
    const wxString lbl_grip_from   = _L("Brace grip from");
    const wxString lbl_tine_spc    = _L("Brace tine spacing");
    const wxString lbl_reach       = _L("Brace depth");
    const wxString lbl_cutout      = _L("Cutouts");
    const wxString lbl_coverage    = _L("Wide-face coverage");
    const wxString lbl_plate_only  = _L("Plate only");

    const float space_size = m_imgui->get_style_scaling() * 8;
    std::vector<wxString> captions = {lbl_mode, lbl_material, lbl_threshold, lbl_tine_grip, lbl_pad, lbl_grip_from, lbl_tine_spc, lbl_reach, lbl_cutout, lbl_coverage};
    const float caption_size  = m_imgui->find_widest_text(captions) + space_size + ImGui::GetStyle().WindowPadding.x;
    const float slider_width  = m_imgui->scaled(8.f);
    const float input_width   = m_imgui->scaled(3.5f);
    const float input_left    = caption_size + slider_width + space_size;
    const float combo_width   = slider_width + space_size + input_width;
    const float wrap_width    = input_left + input_width + m_imgui->scaled(5.f);
    const float max_tip_width = ImGui::GetFontSize() * 20.0f;

    auto show_tooltip = [this, max_tip_width](const wxString &tip) {
        if (!tip.empty() && ImGui::IsItemHovered())
            m_imgui->tooltip(tip, max_tip_width);
    };
    auto show_hint = [this](const wxString &hint) {
        if (hint.empty())
            return;
        ImGui::SameLine();
        m_imgui->text_colored(HINT_TEXT_COLOR, hint);
    };
    // A slider plus a typed field (options.json slider: true). Returns true when an edit was committed.
    auto slider_row = [&](const wxString &label, const std::string &id, float &value, float v_min, float v_max, float step, const char *fmt,
                          const wxString &hint, const wxString &tip) {
        ImGui::AlignTextToFramePadding();
        m_imgui->text(label);
        ImGui::SameLine(caption_size);
        ImGui::SetNextItemWidth(slider_width);
        m_imgui->bbl_slider_float_style("##" + id, &value, v_min, v_max, fmt, 1.0f, true, tip);
        bool committed = m_imgui->get_last_slider_status().deactivated_after_edit;
        ImGui::SameLine(input_left);
        ImGui::SetNextItemWidth(input_width);
        ImGui::BBLDragFloat(("##" + id + "_input").c_str(), &value, step, v_min, v_max, fmt);
        show_tooltip(tip);
        committed |= ImGui::IsItemDeactivatedAfterEdit();
        value = snap_to_step(value, step, v_min, v_max);
        show_hint(hint);
        return committed;
    };
    // A number field (options.json type number without slider).
    auto number_row = [&](const wxString &label, const std::string &id, float &value, float v_min, float v_max, float step, const char *fmt,
                          const wxString &hint, const wxString &tip) {
        ImGui::AlignTextToFramePadding();
        m_imgui->text(label);
        ImGui::SameLine(caption_size);
        ImGui::SetNextItemWidth(input_width);
        ImGui::BBLDragFloat(("##" + id).c_str(), &value, step, v_min, v_max, fmt);
        show_tooltip(tip);
        value = snap_to_step(value, step, v_min, v_max);
        show_hint(hint);
    };
    auto checkbox_row = [&](const wxString &label, bool &value, const wxString &hint, const wxString &tip) {
        m_imgui->bbl_checkbox(label, value);
        show_tooltip(tip);
        show_hint(hint);
    };
    auto combo_row = [&](const wxString &label, const std::vector<std::string> &choices, int &value, const wxString &tip) {
        size_t sel = size_t(std::clamp(value, 0, int(choices.size()) - 1));
        ImGuiWrapper::push_combo_style(currt_scale);
        render_combo(into_u8(label), choices, sel, caption_size, combo_width);
        ImGuiWrapper::pop_combo_style();
        show_tooltip(tip);
        value = int(sel);
    };
    auto section = [&](const wxString &label) {
        ImGui::Separator();
        m_imgui->bold_text(into_u8(label));
    };

    m_imgui->bold_text(_u8L("Support Fins"));

    // Mode: Auto | Draw
    ImGui::AlignTextToFramePadding();
    m_imgui->text(lbl_mode);
    ImGui::SameLine(caption_size);
    if (m_imgui->bbl_radio_button("##sf_mode_auto", m_mode == Mode::Auto) && m_mode != Mode::Auto) {
        m_mode = Mode::Auto;
        m_draw_points.clear();
    }
    ImGui::SameLine();
    m_imgui->text(_L("Auto"));
    ImGui::SameLine();
    if (m_imgui->bbl_radio_button("##sf_mode_draw", m_mode == Mode::Draw) && m_mode != Mode::Draw) {
        m_mode = Mode::Draw;
        m_draw_points.clear();
    }
    ImGui::SameLine();
    m_imgui->text(_L("Draw"));

    // --- Setup
    section(_L("Setup"));
    combo_row(lbl_material, {_u8L("PLA"), _u8L("PETG")}, m_panel.material,
              _L("The filament you'll print in. PETG fuses to supports much harder than PLA, so PETG loosens the gaps and gives the bed pad a gap instead of a tack. PLA keeps the tighter grip."));
    if (slider_row(lbl_threshold, "sf_threshold", m_panel.threshold, 30.f, 70.f, 1.f, "%.0f", _L("°"),
                   _L("Surface angle from the plate below which a face needs support.")))
        // Re-run the overlay analysis only when the drag ends (not on every slider tick).
        m_overlay_threshold = m_panel.threshold;

    // --- Tines
    section(_L("Tines"));
    checkbox_row(lbl_tines, m_panel.tines, _L("grip the part"),
                 _L("Tines fuse the support to the part so it grips instead of just propping. Off = plain breakaway wall."));
    if (m_panel.tines)
        slider_row(lbl_tine_grip, "sf_tine_density", m_panel.tine_density, 0.f, 100.f, 5.f, "%.0f%%", _L("light - firm"),
                   _L("How tightly to space the grip tines. Light = fewest marks (default), a per-wall floor keeps grip; Firm = dense comb / max grip for a tippy or tall part."));
    if (m_panel.tines || m_panel.pad_style == 1 /*auto*/ || m_panel.pad_style == 2 /*light*/) {
        // hostSupplied 'slicer': read from the print settings, never edited here.
        ImGui::AlignTextToFramePadding();
        m_imgui->text_colored(HINT_TEXT_COLOR, wxString::Format(_L("Layer height: %.2f mm (from print settings)"), print_layer_height(*mo)));
        show_tooltip(_L("The grip tines are one layer tall so they snap off clean; they are made for the layer height you slice at."));
    }

    // --- Clearances
    section(_L("Clearances"));
    combo_row(lbl_pad, {_u8L("Off"), _u8L("Auto"), _u8L("Light"), _u8L("Sure hold")}, m_panel.pad_style,
              _L("A tilted part rests on an edge and peels off the plate without a pad. Auto: Light, or Sure hold when the part meets the plate on a point or a small round foot. Light: one layer, 0.12mm off the part, peels off like a brim. Sure hold: thicker and tacked into the part, holds harder but is harder to remove."));

    // --- Sway braces
    section(_L("Sway braces"));
    checkbox_row(lbl_sway, m_panel.sway, _L("tall parts"),
                 _L("Stand tapered buttress ribs against the upright sides of a tall part, tied on with tines all the way up, so it doesn't drift or wobble as it grows."));
    if (m_panel.sway && m_panel.tines) {
        number_row(lbl_grip_from, "sf_sway_grip_from", m_panel.sway_grip_from, 0.f, 2000.f, 5.f, "%.0f", _L("mm up"),
                   _L("Height the brace tines start at. 0 = grip the whole height; raise it to tie on only above where the part starts to move."));
        number_row(lbl_tine_spc, "sf_sway_tine_spacing", m_panel.sway_tine_spacing, 2.f, 30.f, 1.f, "%.0f", _L("mm"),
                   _L("Vertical distance between brace tines. Smaller = held tighter, more marks on the side. Default 6mm."));
    }
    if (m_panel.sway)
        number_row(lbl_reach, "sf_sway_reach", m_panel.sway_reach, 5.f, 50.f, 1.f, "%.0f", _L("% of height"),
                   _L("How far the brace reaches out at the bed, as a share of its height. Deeper = stiffer, more plastic. Default 15%."));

    // --- Walls
    section(_L("Walls"));
    combo_row(lbl_cutout, {_u8L("None"), _u8L("Diamond"), _u8L("Triangle"), _u8L("Arch"), _u8L("Lattice")}, m_panel.cutout,
              _L("Cut holes through tall breakaway walls to save filament. The top edge that holds the part, the foot, and the wall's ends stay solid, and every hole has a pointed roof so it prints without bridging. Short walls stay solid."));
    slider_row(lbl_coverage, "sf_coverage", m_panel.coverage, 0.f, 100.f, 5.f, "%.0f%%", _L("sparse - dense"),
               _L("How densely to line a WIDE overhang face with fins. The middle is the anti-sag default; drag LEFT for fewer fins (a small part can go down to one - the readout warns if a broad face then risks sagging), RIGHT for more support. Narrow parts are unaffected."));
    checkbox_row(lbl_plate_only, m_panel.plate_only, _L("no supports on the part"),
                 _L("Stand every support on the build plate, never on the part itself. An overhang with part under it is left unsupported, and the readout says how many."));

    // --- Overlay legend
    ImGui::Separator();
    m_imgui->text_colored(ImGuiWrapper::to_ImVec4(OVERHANG_COLOR), wxString::Format(_L("Red: overhangs that get fins (%d faces)"), m_red_faces));
    m_imgui->text_colored(ImGuiWrapper::to_ImVec4(TOO_SMALL_COLOR), wxString::Format(_L("Amber: too small or too short to fin (%d faces)"), m_amber_faces));

    // --- Actions
    ImGui::Separator();
    const float f_scale = m_parent.get_main_toolbar_scale();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 4.0f * f_scale));
    if (m_mode == Mode::Auto) {
        if (m_imgui->button(m_has_fins ? _L("Regenerate") : _L("Generate")))
            generate();
        ImGui::SameLine();
    } else {
        m_imgui->text_wrapped(_L("Click two points on an upright side of the part to stand a wall"), wrap_width);
        const bool none_selected = !m_selected_fin.valid();
        m_imgui->disabled_begin(none_selected);
        if (m_imgui->button(_L("Remove selected")))
            remove_selected_fin();
        m_imgui->disabled_end();
        ImGui::SameLine();
    }
    m_imgui->disabled_begin(!m_has_fins);
    if (m_imgui->button(_L("Clear")))
        clear();
    m_imgui->disabled_end();
    ImGui::PopStyleVar(1);

    ImGui::SameLine();
    float get_cur_y = ImGui::GetContentRegionMax().y + ImGui::GetFrameHeight() + y;
    render_tooltip_icon(x, get_cur_y);

    // --- Readout / stale notice
    if (!m_readout.empty()) {
        if (m_readout_is_error)
            m_imgui->warning_text_wrapped(from_u8(m_readout), wrap_width);
        else
            m_imgui->text_wrapped(m_readout, wrap_width);
    }
    if (m_stale)
        m_imgui->warning_text_wrapped(_L("Fins were generated for a different pose. Regenerate."), wrap_width);

    GizmoImguiEnd();
    ImGui::PopStyleVar(1);
    ImGuiWrapper::pop_toolbar_style();
}

}} // namespace Slic3r::GUI
