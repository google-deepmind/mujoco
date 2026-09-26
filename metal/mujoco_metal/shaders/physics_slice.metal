#include <metal_stdlib>
using namespace metal;

// -----------------------------------------------------------------------------
// Constant structures and parameters
// -----------------------------------------------------------------------------

struct BodyConstants {
    int parent_id;
    int joint_type; // 0: free, 3: hinge
    int qpos_adr;
    int dof_adr;
    packed_float3 body_pos;
    packed_float4 body_quat;
    packed_float3 body_ipos;
    packed_float4 body_iquat;
    packed_float3 jnt_axis;
    float mass;
    packed_float3 inertia;
};

struct DofConstants {
    int dof_parentid;
    int dof_bodyid;
    float dof_armature;
};

struct GeomConstants {
    int body_id;
    packed_float3 geom_pos;
    packed_float4 geom_quat;
};

struct vec10 {
    float Ixx, Iyy, Izz;
    float Ixy, Ixz, Iyz;
    float mx, my, mz;
    float m;
};

inline vec10 crb_add(vec10 a, vec10 b) {
    vec10 r;
    r.Ixx = a.Ixx + b.Ixx;
    r.Iyy = a.Iyy + b.Iyy;
    r.Izz = a.Izz + b.Izz;
    r.Ixy = a.Ixy + b.Ixy;
    r.Ixz = a.Ixz + b.Ixz;
    r.Iyz = a.Iyz + b.Iyz;
    r.mx = a.mx + b.mx;
    r.my = a.my + b.my;
    r.mz = a.mz + b.mz;
    r.m = a.m + b.m;
    return r;
}

struct spatial_vec {
    float3 w; // angular
    float3 v; // linear

    spatial_vec() : w(float3(0)), v(float3(0)) {}
    spatial_vec(float3 w_, float3 v_) : w(w_), v(v_) {}
};

inline spatial_vec operator+(spatial_vec a, spatial_vec b) {
    return spatial_vec(a.w + b.w, a.v + b.v);
}

inline spatial_vec operator*(spatial_vec a, float s) {
    return spatial_vec(a.w * s, a.v * s);
}

inline float spatial_dot(spatial_vec a, spatial_vec b) {
    return dot(a.w, b.w) + dot(a.v, b.v);
}

inline spatial_vec inert_vec(vec10 i, spatial_vec v) {
    spatial_vec res;
    res.w.x = i.Ixx * v.w.x + i.Ixy * v.w.y + i.Ixz * v.w.z - i.mz * v.v.y + i.my * v.v.z;
    res.w.y = i.Ixy * v.w.x + i.Iyy * v.w.y + i.Iyz * v.w.z + i.mz * v.v.x - i.mx * v.v.z;
    res.w.z = i.Ixz * v.w.x + i.Iyz * v.w.y + i.Izz * v.w.z - i.my * v.v.x + i.mx * v.v.y;
    res.v.x = i.mz * v.w.y - i.my * v.w.z + i.m * v.v.x;
    res.v.y = i.mx * v.w.z - i.mz * v.w.x + i.m * v.v.y;
    res.v.z = i.my * v.w.x - i.mx * v.w.y + i.m * v.v.z;
    return res;
}

inline spatial_vec motion_cross(spatial_vec u, spatial_vec v) {
    spatial_vec res;
    res.w = cross(u.w, v.w);
    res.v = cross(u.v, v.w) + cross(u.w, v.v);
    return res;
}

inline spatial_vec motion_cross_force(spatial_vec v, spatial_vec f) {
    spatial_vec res;
    res.w = cross(v.w, f.w) + cross(v.v, f.v);
    res.v = cross(v.w, f.v);
    return res;
}

// -----------------------------------------------------------------------------
// Math helper functions
// -----------------------------------------------------------------------------

inline float4 quat_mul(float4 q1, float4 q2) {
    return float4(
        q1.x * q2.x - q1.y * q2.y - q1.z * q2.z - q1.w * q2.w,
        q1.x * q2.y + q1.y * q2.x + q1.z * q2.w - q1.w * q2.z,
        q1.x * q2.z - q1.y * q2.w + q1.z * q2.x + q1.w * q2.y,
        q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x
    );
}

inline float3 quat_rot(float4 q, float3 v) {
    float3 q_vec = float3(q.y, q.z, q.w);
    float3 t = 2.0f * cross(q_vec, v);
    return v + q.x * t + cross(q_vec, t);
}

inline float3x3 quat_to_mat(float4 q) {
    float w = q.x, x = q.y, y = q.z, z = q.w;
    return float3x3(
        float3(1.0f - 2.0f*(y*y + z*z), 2.0f*(x*y + w*z), 2.0f*(x*z - w*y)), // col 0
        float3(2.0f*(x*y - w*z), 1.0f - 2.0f*(x*x + z*z), 2.0f*(y*z + w*x)), // col 1
        float3(2.0f*(x*z + w*y), 2.0f*(y*z - w*x), 1.0f - 2.0f*(x*x + y*y))  // col 2
    );
}

inline float3x3 load_row_major_mat3(device const float* ptr) {
    return float3x3(
        float3(ptr[0], ptr[3], ptr[6]), // col 0
        float3(ptr[1], ptr[4], ptr[7]), // col 1
        float3(ptr[2], ptr[5], ptr[8])  // col 2
    );
}

// -----------------------------------------------------------------------------
// 1. Hierarchical Forward Kinematics Kernel
// -----------------------------------------------------------------------------

kernel void kernel_forward_kinematics(
    constant BodyConstants* bodies [[buffer(0)]],      // 17 bodies
    constant GeomConstants* foot_geoms [[buffer(1)]],  // 2 foot geoms (left, right)
    device const float* qpos_batch [[buffer(2)]],      // (B, 21)
    device float* xpos_out [[buffer(3)]],              // (B, 17, 3)
    device float* xmat_out [[buffer(4)]],              // (B, 17, 9) row-major
    device float* geom_xpos_out [[buffer(5)]],         // (B, 2, 3)
    device float* geom_xmat_out [[buffer(6)]],         // (B, 2, 9) row-major
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device const float* qpos = qpos_batch + b_idx * 21;
    device float* b_xpos = xpos_out + b_idx * 17 * 3;
    device float* b_xmat = xmat_out + b_idx * 17 * 9;
    device float* g_xpos = geom_xpos_out + b_idx * 2 * 3;
    device float* g_xmat = geom_xmat_out + b_idx * 2 * 9;

    float3 xpos[17];
    float4 xquat[17];

    // Body 0: world
    xpos[0] = float3(0.0f);
    xquat[0] = float4(1.0f, 0.0f, 0.0f, 0.0f);

    // Body 1: terrain
    xpos[1] = float3(0.0f);
    xquat[1] = float4(1.0f, 0.0f, 0.0f, 0.0f);

    // Body 2: trunk_base (freejoint)
    xpos[2] = float3(qpos[0], qpos[1], qpos[2]);
    float4 q_raw = float4(qpos[3], qpos[4], qpos[5], qpos[6]);
    xquat[2] = normalize(q_raw);

    // Bodies 3..16 in topological order
    for (int i = 3; i < 17; ++i) {
        int pid = bodies[i].parent_id;
        float3 pos_rel = float3(bodies[i].body_pos);
        float4 quat_rel = float4(bodies[i].body_quat);
        float3 axis = float3(bodies[i].jnt_axis);
        int qadr = bodies[i].qpos_adr;
        float angle = qpos[qadr];

        float half_a = angle * 0.5f;
        float4 q_jnt = float4(cos(half_a), axis * sin(half_a));
        float4 q_b = quat_mul(quat_rel, q_jnt);

        xquat[i] = normalize(quat_mul(xquat[pid], q_b));
        xpos[i] = xpos[pid] + quat_rot(xquat[pid], pos_rel);
    }

    // Write body xpos and row-major xmat
    for (int i = 0; i < 17; ++i) {
        b_xpos[i * 3 + 0] = xpos[i].x;
        b_xpos[i * 3 + 1] = xpos[i].y;
        b_xpos[i * 3 + 2] = xpos[i].z;

        float3x3 m = quat_to_mat(xquat[i]);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                b_xmat[i * 9 + r * 3 + c] = m[c][r]; // row r, col c
            }
        }
    }

    // Compute foot geom frames (0: left foot, 1: right foot)
    for (int g = 0; g < 2; ++g) {
        int bid = foot_geoms[g].body_id;
        float3 g_rel_pos = float3(foot_geoms[g].geom_pos);
        float4 g_rel_quat = float4(foot_geoms[g].geom_quat);

        float3 pos = xpos[bid] + quat_rot(xquat[bid], g_rel_pos);
        float4 quat = normalize(quat_mul(xquat[bid], g_rel_quat));
        float3x3 m = quat_to_mat(quat);

        g_xpos[g * 3 + 0] = pos.x;
        g_xpos[g * 3 + 1] = pos.y;
        g_xpos[g * 3 + 2] = pos.z;

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                g_xmat[g * 9 + r * 3 + c] = m[c][r]; // row r, col c
            }
        }
    }
}

// -----------------------------------------------------------------------------
// 1b. Native Articulated Dynamics Kernel (CRBA + RNE + Per-World Randomization)
// -----------------------------------------------------------------------------

kernel void kernel_articulated_dynamics(
    constant BodyConstants* bodies [[buffer(0)]],        // 17 bodies
    constant DofConstants* dofs [[buffer(1)]],           // 20 dofs
    device const float* qpos_batch [[buffer(2)]],        // (B, 21)
    device const float* qvel_batch [[buffer(3)]],        // (B, 20)
    device const float* per_world_mass [[buffer(4)]],    // (B, 17) optional
    device const float* per_world_ipos [[buffer(5)]],    // (B, 17, 3) optional
    device const float* per_world_armature [[buffer(6)]],// (B, 20) optional
    constant int& flags [[buffer(7)]],                   // bit 0: mass, bit 1: ipos, bit 2: armature, bit 3: inertia, bit 4: damping
    device float* M_eff_out [[buffer(8)]],               // (B, 20, 20)
    device float* qfrc_bias_out [[buffer(9)]],           // (B, 20)
    device float* xpos_out [[buffer(10)]],               // (B, 17, 3)
    device float* xmat_out [[buffer(11)]],               // (B, 17, 9) row-major
    device float* xipos_out [[buffer(12)]],              // (B, 17, 3)
    device float* ximat_out [[buffer(13)]],              // (B, 17, 9) row-major
    device float* subtree_com_out [[buffer(14)]],        // (B, 3)
    device const float* per_world_inertia [[buffer(15)]],// (B, 17, 3) optional
    device const float* per_world_damping [[buffer(16)]],// (B, 20) optional
    constant float& timestep [[buffer(17)]],             // dt
    device const float* per_world_iquat [[buffer(18)]],  // (B, 17, 4) optional
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device const float* qpos = qpos_batch + b_idx * 21;
    device const float* qvel = qvel_batch + b_idx * 20;
    device float* M_out = M_eff_out + b_idx * 400;
    device float* bias_out = qfrc_bias_out + b_idx * 20;

    // Per-world parameters
    float mass[17];
    float3 ipos[17];
    float3 inertia[17];
    float armature[20];
    float damping[20];

    for (int i = 0; i < 17; ++i) {
        mass[i] = (flags & 1) ? per_world_mass[b_idx * 17 + i] : bodies[i].mass;
        if (flags & 2) {
            ipos[i] = float3(
                per_world_ipos[(b_idx * 17 + i) * 3 + 0],
                per_world_ipos[(b_idx * 17 + i) * 3 + 1],
                per_world_ipos[(b_idx * 17 + i) * 3 + 2]
            );
        } else {
            ipos[i] = float3(bodies[i].body_ipos);
        }
        if (flags & 8) {
            inertia[i] = float3(
                per_world_inertia[(b_idx * 17 + i) * 3 + 0],
                per_world_inertia[(b_idx * 17 + i) * 3 + 1],
                per_world_inertia[(b_idx * 17 + i) * 3 + 2]
            );
        } else {
            inertia[i] = float3(bodies[i].inertia[0], bodies[i].inertia[1], bodies[i].inertia[2]);
        }
    }
    for (int d = 0; d < 20; ++d) {
        armature[d] = (flags & 4) ? per_world_armature[b_idx * 20 + d] : dofs[d].dof_armature;
        damping[d] = (flags & 16) ? per_world_damping[b_idx * 20 + d] : 0.0f;
    }

    // Kinematics arrays
    float3 xpos[17];
    float4 xquat[17];
    float3x3 xmat[17];
    float3 xipos[17];
    float3x3 ximat[17];
    float3 xaxis[20];
    float3 xanchor[20];

    xpos[0] = float3(0.0f);
    xquat[0] = float4(1.0f, 0.0f, 0.0f, 0.0f);
    xmat[0] = float3x3(1.0f);
    xipos[0] = float3(0.0f);
    ximat[0] = float3x3(1.0f);

    xpos[1] = float3(0.0f);
    xquat[1] = float4(1.0f, 0.0f, 0.0f, 0.0f);
    xmat[1] = float3x3(1.0f);
    xipos[1] = float3(0.0f);
    ximat[1] = float3x3(1.0f);

    // Body 2 (trunk_base, freejoint)
    xpos[2] = float3(qpos[0], qpos[1], qpos[2]);
    xquat[2] = normalize(float4(qpos[3], qpos[4], qpos[5], qpos[6]));
    xmat[2] = quat_to_mat(xquat[2]);
    xipos[2] = xpos[2] + quat_rot(xquat[2], ipos[2]);
    float4 iquat2 = (flags & 32) ? float4(
        per_world_iquat[(b_idx * 17 + 2) * 4 + 0],
        per_world_iquat[(b_idx * 17 + 2) * 4 + 1],
        per_world_iquat[(b_idx * 17 + 2) * 4 + 2],
        per_world_iquat[(b_idx * 17 + 2) * 4 + 3]
    ) : float4(bodies[2].body_iquat);
    ximat[2] = quat_to_mat(normalize(quat_mul(xquat[2], iquat2)));

    for (int k = 0; k < 6; ++k) {
        xanchor[k] = xpos[2];
    }

    // Bodies 3..16 (topological order)
    for (int i = 3; i < 17; ++i) {
        int pid = bodies[i].parent_id;
        float3 pos_rel = float3(bodies[i].body_pos);
        float4 quat_rel = float4(bodies[i].body_quat);
        float3 axis = float3(bodies[i].jnt_axis);
        int qadr = bodies[i].qpos_adr;
        int dof = bodies[i].dof_adr;
        float angle = qpos[qadr];

        float half_a = angle * 0.5f;
        float4 q_jnt = float4(cos(half_a), axis * sin(half_a));
        float4 q_b = quat_mul(quat_rel, q_jnt);

        xquat[i] = normalize(quat_mul(xquat[pid], q_b));
        xanchor[dof] = xpos[pid] + quat_rot(xquat[pid], pos_rel);
        xpos[i] = xanchor[dof]; // jnt_pos = 0
        xmat[i] = quat_to_mat(xquat[i]);
        xipos[i] = xpos[i] + quat_rot(xquat[i], ipos[i]);
        float4 iquati = (flags & 32) ? float4(
            per_world_iquat[(b_idx * 17 + i) * 4 + 0],
            per_world_iquat[(b_idx * 17 + i) * 4 + 1],
            per_world_iquat[(b_idx * 17 + i) * 4 + 2],
            per_world_iquat[(b_idx * 17 + i) * 4 + 3]
        ) : float4(bodies[i].body_iquat);
        ximat[i] = quat_to_mat(normalize(quat_mul(xquat[i], iquati)));
        xaxis[dof] = quat_rot(xquat[i], axis);
    }

    // Subtree Center of Mass (robot root is body 2)
    float tot_mass = 0.0f;
    float3 com_num = float3(0.0f);
    for (int i = 2; i < 17; ++i) {
        tot_mass += mass[i];
        com_num += mass[i] * xipos[i];
    }
    float3 subtree_com = (tot_mass > 0.0f) ? (com_num / tot_mass) : float3(0.0f);

    // Spatial Inertias (cinert) in subtree CoM frame
    vec10 cinert[17];
    for (int i = 2; i < 17; ++i) {
        float3 dif = xipos[i] - subtree_com;
        float3x3 mat = ximat[i];
        float3 inert = inertia[i];
        float3x3 diag_inert = float3x3(
            float3(inert.x, 0, 0),
            float3(0, inert.y, 0),
            float3(0, 0, inert.z)
        );
        float3x3 tmp = mat * diag_inert * transpose(mat);

        vec10 ci;
        ci.Ixx = tmp[0][0] + mass[i] * (dif.y * dif.y + dif.z * dif.z);
        ci.Iyy = tmp[1][1] + mass[i] * (dif.x * dif.x + dif.z * dif.z);
        ci.Izz = tmp[2][2] + mass[i] * (dif.x * dif.x + dif.y * dif.y);
        ci.Ixy = tmp[0][1] - mass[i] * dif.x * dif.y;
        ci.Ixz = tmp[0][2] - mass[i] * dif.x * dif.z;
        ci.Iyz = tmp[1][2] - mass[i] * dif.y * dif.z;
        ci.mx = mass[i] * dif.x;
        ci.my = mass[i] * dif.y;
        ci.mz = mass[i] * dif.z;
        ci.m = mass[i];
        cinert[i] = ci;
    }

    // Spatial Motion DOFs (cdof) in subtree CoM frame
    spatial_vec cdof[20];
    cdof[0] = spatial_vec(float3(0), float3(1, 0, 0));
    cdof[1] = spatial_vec(float3(0), float3(0, 1, 0));
    cdof[2] = spatial_vec(float3(0), float3(0, 0, 1));

    float3 offset_root = subtree_com - xanchor[0];
    cdof[3] = spatial_vec(xmat[2][0], cross(xmat[2][0], offset_root));
    cdof[4] = spatial_vec(xmat[2][1], cross(xmat[2][1], offset_root));
    cdof[5] = spatial_vec(xmat[2][2], cross(xmat[2][2], offset_root));

    for (int d = 6; d < 20; ++d) {
        float3 offset = subtree_com - xanchor[d];
        float3 ax = xaxis[d];
        cdof[d] = spatial_vec(ax, cross(ax, offset));
    }

    // CRBA: Composite Rigid Body Inertias
    vec10 crb[17];
    for (int b = 2; b < 17; ++b) {
        crb[b] = cinert[b];
    }
    for (int b = 16; b >= 3; --b) {
        int pid = bodies[b].parent_id;
        crb[pid] = crb_add(crb[pid], crb[b]);
    }

    // Form M(q) including configured armature
    for (int i = 0; i < 400; ++i) {
        M_out[i] = 0.0f;
    }

    for (int i = 0; i < 20; ++i) {
        int bid = dofs[i].dof_bodyid;
        spatial_vec buf = inert_vec(crb[bid], cdof[i]);
        float diag_val = armature[i] + spatial_dot(cdof[i], buf);
        M_out[i * 20 + i] = diag_val;

        int j = dofs[i].dof_parentid;
        while (j >= 0) {
            float val = spatial_dot(cdof[j], buf);
            M_out[i * 20 + j] = val;
            M_out[j * 20 + i] = val;
            j = dofs[j].dof_parentid;
        }
    }

    // RNE: Coriolis, centrifugal, gravity bias forces
    spatial_vec cvel[17];
    cvel[0] = spatial_vec(float3(0), float3(0));
    cvel[1] = spatial_vec(float3(0), float3(0));
    cvel[2] = spatial_vec(float3(0), float3(0));
    for (int k = 0; k < 6; ++k) {
        cvel[2] = cvel[2] + cdof[k] * qvel[k];
    }
    for (int b = 3; b < 17; ++b) {
        int pid = bodies[b].parent_id;
        int dof = bodies[b].dof_adr;
        cvel[b] = cvel[pid] + cdof[dof] * qvel[dof];
    }

    spatial_vec cdof_dot[20];
    cdof_dot[0] = spatial_vec(float3(0), float3(0));
    cdof_dot[1] = spatial_vec(float3(0), float3(0));
    cdof_dot[2] = spatial_vec(float3(0), float3(0));
    for (int k = 3; k < 6; ++k) {
        cdof_dot[k] = motion_cross(cvel[2], cdof[k]);
    }
    for (int d = 6; d < 20; ++d) {
        int b = dofs[d].dof_bodyid;
        cdof_dot[d] = motion_cross(cvel[b], cdof[d]);
    }

    spatial_vec cacc[17];
    cacc[0] = spatial_vec(float3(0), float3(0, 0, 9.81f)); // -gravity
    cacc[1] = cacc[0];
    cacc[2] = cacc[0];
    for (int k = 0; k < 6; ++k) {
        cacc[2] = cacc[2] + cdof_dot[k] * qvel[k];
    }
    for (int b = 3; b < 17; ++b) {
        int pid = bodies[b].parent_id;
        int dof = bodies[b].dof_adr;
        cacc[b] = cacc[pid] + cdof_dot[dof] * qvel[dof];
    }

    spatial_vec cfrc[17];
    cfrc[0] = spatial_vec(float3(0), float3(0));
    cfrc[1] = spatial_vec(float3(0), float3(0));
    for (int b = 2; b < 17; ++b) {
        spatial_vec iv_acc = inert_vec(cinert[b], cacc[b]);
        spatial_vec iv_vel = inert_vec(cinert[b], cvel[b]);
        cfrc[b] = iv_acc + motion_cross_force(cvel[b], iv_vel);
    }

    for (int b = 16; b >= 3; --b) {
        int pid = bodies[b].parent_id;
        cfrc[pid] = cfrc[pid] + cfrc[b];
    }

    for (int d = 0; d < 20; ++d) {
        int b = dofs[d].dof_bodyid;
        bias_out[d] = spatial_dot(cdof[d], cfrc[b]);
    }

    // Write auxiliary kinematics outputs to device buffers
    device float* b_xpos = xpos_out + b_idx * 17 * 3;
    device float* b_xmat = xmat_out + b_idx * 17 * 9;
    device float* b_xipos = xipos_out + b_idx * 17 * 3;
    device float* b_ximat = ximat_out + b_idx * 17 * 9;
    device float* b_com = subtree_com_out + b_idx * 3;

    b_com[0] = subtree_com.x;
    b_com[1] = subtree_com.y;
    b_com[2] = subtree_com.z;

    for (int i = 0; i < 17; ++i) {
        b_xpos[i * 3 + 0] = xpos[i].x;
        b_xpos[i * 3 + 1] = xpos[i].y;
        b_xpos[i * 3 + 2] = xpos[i].z;

        b_xipos[i * 3 + 0] = xipos[i].x;
        b_xipos[i * 3 + 1] = xipos[i].y;
        b_xipos[i * 3 + 2] = xipos[i].z;

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                b_xmat[i * 9 + r * 3 + c] = xmat[i][c][r]; // row-major
                b_ximat[i * 9 + r * 3 + c] = ximat[i][c][r]; // row-major
            }
        }
    }
}

// -----------------------------------------------------------------------------
// 2. CAD Sole Contact Manifold Kernel
// -----------------------------------------------------------------------------

kernel void kernel_cad_contact_manifold(
    device const float* geom_xpos_batch [[buffer(0)]],      // (B, 2, 3)
    device const float* geom_xmat_batch [[buffer(1)]],      // (B, 2, 9) row-major
    constant const float* left_mesh_verts [[buffer(2)]],    // (N_L, 3)
    constant const float* right_mesh_verts [[buffer(3)]],   // (N_R, 3)
    constant int& num_left_verts [[buffer(4)]],             // N_L
    constant int& num_right_verts [[buffer(5)]],            // N_R
    device float* contact_pos_out [[buffer(6)]],            // (B, nconmax, 3)
    device float* contact_dist_out [[buffer(7)]],           // (B, nconmax)
    device float* contact_normal_out [[buffer(8)]],         // (B, nconmax, 3)
    device int* contact_body_out [[buffer(9)]],             // (B, nconmax) body ID for contact
    device int* ncon_out [[buffer(10)]],                    // (B,)
    device int* overflow_flag_out [[buffer(11)]],           // (B,)
    constant int& nconmax [[buffer(12)]],                   // capacity
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device const float* g_xpos = geom_xpos_batch + b_idx * 2 * 3;
    device const float* g_xmat = geom_xmat_batch + b_idx * 2 * 9;

    device float* out_pos = contact_pos_out + b_idx * nconmax * 3;
    device float* out_dist = contact_dist_out + b_idx * nconmax;
    device float* out_norm = contact_normal_out + b_idx * nconmax * 3;
    device int* out_body = contact_body_out + b_idx * nconmax;

    int total_contacts = 0;
    int overflow = 0;

    // Process left foot (geom 0, body 7) and right foot (geom 1, body 16)
    for (int foot = 0; foot < 2; ++foot) {
        float3 pos_g = float3(g_xpos[foot * 3 + 0], g_xpos[foot * 3 + 1], g_xpos[foot * 3 + 2]);
        float3x3 mat_g = load_row_major_mat3(g_xmat + foot * 9);

        constant const float* verts = (foot == 0) ? left_mesh_verts : right_mesh_verts;
        int nverts = (foot == 0) ? num_left_verts : num_right_verts;
        int body_id = (foot == 0) ? 7 : 16;

        // 1. Find deepest penetrating vertex (point a)
        float min_z = 1e6f;
        int idx_a = -1;
        float3 w_a = float3(0.0f);

        for (int v = 0; v < nverts; ++v) {
            float3 local_v = float3(verts[v * 3 + 0], verts[v * 3 + 1], verts[v * 3 + 2]);
            float3 w_v = pos_g + mat_g * local_v;
            if (w_v.z < min_z) {
                min_z = w_v.z;
                idx_a = v;
                w_a = w_v;
            }
        }

        // If foot does not penetrate ground plane (z=0), skip
        if (min_z >= 0.0f || idx_a < 0) {
            continue;
        }

        // Support threshold (1 mm above deepest)
        float threshold = min_z + 1e-3f;

        // 2. Find vertex b furthest from a in xy
        float max_d_ab = -1e6f;
        int idx_b = -1;
        float3 w_b = w_a;

        for (int v = 0; v < nverts; ++v) {
            float3 local_v = float3(verts[v * 3 + 0], verts[v * 3 + 1], verts[v * 3 + 2]);
            float3 w_v = pos_g + mat_g * local_v;
            if (w_v.z <= threshold) {
                float d2 = (w_v.x - w_a.x)*(w_v.x - w_a.x) + (w_v.y - w_a.y)*(w_v.y - w_a.y);
                if (d2 > max_d_ab) {
                    max_d_ab = d2;
                    idx_b = v;
                    w_b = w_v;
                }
            }
        }

        // 3. Find vertex c furthest from line a-b
        float2 ab = float2(w_b.x - w_a.x, w_b.y - w_a.y);
        float ab_len = length(ab);
        float max_d_c = -1e6f;
        int idx_c = -1;
        float3 w_c = w_a;

        if (ab_len > 1e-4f) {
            float2 ab_unit = ab / ab_len;
            float2 perp = float2(-ab_unit.y, ab_unit.x);
            for (int v = 0; v < nverts; ++v) {
                float3 local_v = float3(verts[v * 3 + 0], verts[v * 3 + 1], verts[v * 3 + 2]);
                float3 w_v = pos_g + mat_g * local_v;
                if (w_v.z <= threshold) {
                    float dist_line = abs(dot(float2(w_v.x - w_a.x, w_v.y - w_a.y), perp));
                    if (dist_line > max_d_c) {
                        max_d_c = dist_line;
                        idx_c = v;
                        w_c = w_v;
                    }
                }
            }
        }

        // Assemble unique contact points for this foot (up to 3)
        float3 foot_pts[3];
        float foot_dists[3];
        int num_foot_con = 0;

        foot_pts[num_foot_con] = float3(w_a.x, w_a.y, w_a.z * 0.5f);
        foot_dists[num_foot_con] = w_a.z;
        num_foot_con++;

        if (idx_b >= 0 && idx_b != idx_a && max_d_ab > 1e-6f) {
            foot_pts[num_foot_con] = float3(w_b.x, w_b.y, w_b.z * 0.5f);
            foot_dists[num_foot_con] = w_b.z;
            num_foot_con++;
        }

        if (idx_c >= 0 && idx_c != idx_a && idx_c != idx_b && max_d_c > 1e-4f) {
            foot_pts[num_foot_con] = float3(w_c.x, w_c.y, w_c.z * 0.5f);
            foot_dists[num_foot_con] = w_c.z;
            num_foot_con++;
        }

        // Store into batch buffer with explicit capacity check
        for (int c = 0; c < num_foot_con; ++c) {
            if (total_contacts < nconmax) {
                out_pos[total_contacts * 3 + 0] = foot_pts[c].x;
                out_pos[total_contacts * 3 + 1] = foot_pts[c].y;
                out_pos[total_contacts * 3 + 2] = foot_pts[c].z;

                out_dist[total_contacts] = foot_dists[c];

                out_norm[total_contacts * 3 + 0] = 0.0f;
                out_norm[total_contacts * 3 + 1] = 0.0f;
                out_norm[total_contacts * 3 + 2] = 1.0f;

                out_body[total_contacts] = body_id;
                total_contacts++;
            } else {
                overflow = 1;
            }
        }
    }

    ncon_out[b_idx] = total_contacts;
    overflow_flag_out[b_idx] = overflow;
}

// -----------------------------------------------------------------------------
// 2b. CAD Sole Contact Manifold Kernel V2 (Pinned mjc_PlaneConvex)
// -----------------------------------------------------------------------------

struct ContactSolverParams {
    float timeconst;
    float dampratio;
    float dmin;
    float dmax;
    float width;
    float midpoint;
    float power;
    float impratio;
    float margin;
};

kernel void kernel_cad_contact_manifold_v2(
    device const float* geom_xpos_batch [[buffer(0)]],      // (B, 2, 3) foot geoms (0: left, 1: right)
    device const float* geom_xmat_batch [[buffer(1)]],      // (B, 2, 9) row-major
    device const float* left_mesh_verts [[buffer(2)]],      // (N_L, 3)
    device const float* right_mesh_verts [[buffer(3)]],     // (N_R, 3)
    device const int* left_mesh_graph [[buffer(4)]],        // int buffer
    device const int* right_mesh_graph [[buffer(5)]],       // int buffer
    constant float& left_rbound [[buffer(6)]],
    constant float& right_rbound [[buffer(7)]],
    device float* contact_pos_out [[buffer(8)]],            // (B, stride_nconmax, 3)
    device float* contact_dist_out [[buffer(9)]],           // (B, stride_nconmax)
    device float* contact_normal_out [[buffer(10)]],        // (B, stride_nconmax, 3)
    device int* contact_body_out [[buffer(11)]],            // (B, stride_nconmax) body ID (7 or 16)
    device int* contact_geom_out [[buffer(12)]],            // (B, stride_nconmax) geom ID (1 or 2)
    device int* ncon_out [[buffer(13)]],                    // (B,)
    device int* overflow_flag_out [[buffer(14)]],           // (B,)
    constant int& stride_nconmax [[buffer(15)]],            // buffer allocation stride
    constant int& active_nconmax [[buffer(16)]],            // active capacity limit
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device const float* g_xpos = geom_xpos_batch + b_idx * 2 * 3;
    device const float* g_xmat = geom_xmat_batch + b_idx * 2 * 9;

    device float* out_pos = contact_pos_out + b_idx * stride_nconmax * 3;
    device float* out_dist = contact_dist_out + b_idx * stride_nconmax;
    device float* out_norm = contact_normal_out + b_idx * stride_nconmax * 3;
    device int* out_body = contact_body_out + b_idx * stride_nconmax;
    device int* out_geom = contact_geom_out + b_idx * stride_nconmax;

    // Non-finite input guard
    bool finite_geom = true;
    for (int i = 0; i < 2 * 3; ++i) {
        if (!isfinite(g_xpos[i])) { finite_geom = false; break; }
    }
    for (int i = 0; i < 2 * 9; ++i) {
        if (!isfinite(g_xmat[i])) { finite_geom = false; break; }
    }
    if (!finite_geom) {
        ncon_out[b_idx] = 0;
        overflow_flag_out[b_idx] = -1; // Non-finite input error
        for (int c = 0; c < stride_nconmax; ++c) {
            out_pos[c * 3 + 0] = NAN;
            out_pos[c * 3 + 1] = NAN;
            out_pos[c * 3 + 2] = NAN;
            out_dist[c] = NAN;
            out_norm[c * 3 + 0] = NAN;
            out_norm[c * 3 + 1] = NAN;
            out_norm[c * 3 + 2] = NAN;
            out_body[c] = -1;
            out_geom[c] = -1;
        }
        return;
    }

    int total_contacts = 0;
    int overflow = 0;
    int max_contacts = min(stride_nconmax, active_nconmax);

    float3 pos1 = float3(0.0f, 0.0f, 0.0f);
    float3 normal = float3(0.0f, 0.0f, 1.0f);
    float3 ccd_dir = float3(0.0f, 0.0f, -1.0f);
    float margin = 0.0f;

    // Process left foot (foot 0, body 7, geom 1) then right foot (foot 1, body 16, geom 2)
    for (int foot = 0; foot < 2; ++foot) {
        float3 pos2 = float3(g_xpos[foot * 3 + 0], g_xpos[foot * 3 + 1], g_xpos[foot * 3 + 2]);
        float3x3 mat2 = load_row_major_mat3(g_xmat + foot * 9);

        device const float* verts = (foot == 0) ? left_mesh_verts : right_mesh_verts;
        device const int* graph = (foot == 0) ? left_mesh_graph : right_mesh_graph;
        float rbound = (foot == 0) ? left_rbound : right_rbound;
        int body_id = (foot == 0) ? 7 : 16;
        int geom_id = (foot == 0) ? 1 : 2;

        // Direction in geom local frame: transpose(mat2) * ccd_dir
        float3 locdir = transpose(mat2) * ccd_dir;

        int numvert = graph[0];
        device const int* vert_edgeadr = graph + 2;
        device const int* vert_globalid = graph + 2 + numvert;
        device const int* edge_localid = graph + 2 + 2 * numvert;

        // Hill climb on convex hull graph to find support vertex
        int ibest = 0;
        int v0_gid = vert_globalid[0];
        float3 v0 = float3(verts[3 * v0_gid + 0], verts[3 * v0_gid + 1], verts[3 * v0_gid + 2]);
        float tmp = dot(v0, locdir);

        bool change = true;
        int climb_iters = 0;
        while (change && climb_iters < 300) {
            climb_iters++;
            change = false;
            int i = vert_edgeadr[ibest];
            while (edge_localid[i] >= 0) {
                int locid = edge_localid[i];
                int v_gid = vert_globalid[locid];
                float3 v = float3(verts[3 * v_gid + 0], verts[3 * v_gid + 1], verts[3 * v_gid + 2]);
                float vdot = dot(v, locdir);
                if (vdot > tmp) {
                    tmp = vdot;
                    ibest = locid;
                    change = true;
                }
                i++;
            }
        }

        // Primary support contact
        int ibest_global = vert_globalid[ibest];
        float3 best_v = float3(verts[3 * ibest_global + 0], verts[3 * ibest_global + 1], verts[3 * ibest_global + 2]);
        float3 best_world = pos2 + mat2 * best_v;
        float3 dif0 = best_world - pos1;
        float dist0 = dot(normal, dif0);

        if (dist0 <= margin) {
            float3 pos0 = best_world - 0.5f * dist0 * normal;

            float3 foot_pts[3];
            float foot_dists[3];
            foot_pts[0] = pos0;
            foot_dists[0] = dist0;
            int foot_count = 1;

            // Look for additional contacts in ibest neighborhood
            float threshold = dot(normal, pos2 - pos1) - margin;
            float tol_dist = 0.3f * rbound;

            int i = vert_edgeadr[ibest];
            while (edge_localid[i] >= 0 && foot_count < 3) {
                int locid = edge_localid[i];
                int v_gid = vert_globalid[locid];
                float3 v = float3(verts[3 * v_gid + 0], verts[3 * v_gid + 1], verts[3 * v_gid + 2]);
                float vdot = dot(v, locdir);
                if (vdot > threshold) {
                    float3 pnt = pos2 + mat2 * v;
                    if (distance(pnt, pos0) >= tol_dist) {
                        float3 dif = pnt - pos1;
                        float c_dist = dot(normal, dif);
                        float3 c_pos = pnt - 0.5f * c_dist * normal;
                        foot_pts[foot_count] = c_pos;
                        foot_dists[foot_count] = c_dist;
                        foot_count++;
                    }
                }
                i++;
            }

            // Store into batch buffer
            for (int c = 0; c < foot_count; ++c) {
                if (total_contacts < max_contacts) {
                    out_pos[total_contacts * 3 + 0] = foot_pts[c].x;
                    out_pos[total_contacts * 3 + 1] = foot_pts[c].y;
                    out_pos[total_contacts * 3 + 2] = foot_pts[c].z;

                    out_dist[total_contacts] = foot_dists[c];

                    out_norm[total_contacts * 3 + 0] = 0.0f;
                    out_norm[total_contacts * 3 + 1] = 0.0f;
                    out_norm[total_contacts * 3 + 2] = 1.0f;

                    out_body[total_contacts] = body_id;
                    out_geom[total_contacts] = geom_id;
                    total_contacts++;
                } else {
                    overflow = 1;
                }
            }
        }
    }

    // Zero out unused slots up to stride_nconmax
    for (int c = total_contacts; c < stride_nconmax; ++c) {
        out_pos[c * 3 + 0] = 0.0f;
        out_pos[c * 3 + 1] = 0.0f;
        out_pos[c * 3 + 2] = 0.0f;
        out_dist[c] = 0.0f;
        out_norm[c * 3 + 0] = 0.0f;
        out_norm[c * 3 + 1] = 0.0f;
        out_norm[c * 3 + 2] = 0.0f;
        out_body[c] = 0;
        out_geom[c] = 0;
    }

    ncon_out[b_idx] = total_contacts;
    overflow_flag_out[b_idx] = overflow;
}

// -----------------------------------------------------------------------------
// 2c. Contact Constraint Assembly Kernel (Exact MuJoCo 3.10.0 formulas)
// -----------------------------------------------------------------------------

constant float JNT_RANGE_MIN[14] = {
    -0.436332f, -0.383972f, -1.5708f, -1.5708f, -1.5708f, -1.5708f, -1.5708f,
    -2.96706f,  -0.436332f, -0.523599f, -0.383972f, -1.5708f, -1.5708f, -1.5708f
};
constant float JNT_RANGE_MAX[14] = {
     0.523599f,  0.383972f,  1.5708f,  1.5708f,  1.5708f,  1.0472f,  1.5708f,
     2.96706f,   0.436332f,  0.436332f,  0.383972f,  1.5708f,  1.5708f,  1.5708f
};
constant float DOF_INVWEIGHT0[14] = {
    513.038221631049f, 483.78804745f, 495.8380935f,  528.08185897f,
    549.31014583f,     466.6075101f,  498.553959f,   489.21479826f,
    513.28994847f,     513.03480895f, 483.76095898f, 495.81065217f,
    528.07892273f,     549.31034035f
};

inline void compute_body_point_jacobian(
    int body_id,
    float3 p,
    float3 base_pos,
    float3x3 base_mat,
    device const float* b_xpos,
    device const float* b_xmat,
    constant BodyConstants* bodies,
    thread float J_p[3][20]
) {
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 20; ++col) {
            J_p[r][col] = 0.0f;
        }
    }
    if (body_id <= 1) {
        return; // World / terrain: stationary
    }

    // Freejoint linear DOFs (0, 1, 2)
    J_p[0][0] = 1.0f;
    J_p[1][1] = 1.0f;
    J_p[2][2] = 1.0f;

    // Freejoint angular DOFs (3, 4, 5): col_j = base_mat[:, j] x (p - base_pos)
    float3 r_pt = p - base_pos;
    float3 col3 = cross(base_mat[0], r_pt);
    float3 col4 = cross(base_mat[1], r_pt);
    float3 col5 = cross(base_mat[2], r_pt);

    J_p[0][3] = col3.x;  J_p[0][4] = col4.x;  J_p[0][5] = col5.x;
    J_p[1][3] = col3.y;  J_p[1][4] = col4.y;  J_p[1][5] = col5.y;
    J_p[2][3] = col3.z;  J_p[2][4] = col4.z;  J_p[2][5] = col5.z;

    // Ancestor hinge joints
    int curr_b = body_id;
    while (curr_b > 2) {
        int dof_adr = bodies[curr_b].dof_adr;
        float3 local_axis = float3(bodies[curr_b].jnt_axis[0], bodies[curr_b].jnt_axis[1], bodies[curr_b].jnt_axis[2]);
        float3x3 mat_b = load_row_major_mat3(b_xmat + curr_b * 9);
        float3 world_axis = mat_b * local_axis;
        float3 anchor = float3(b_xpos[curr_b * 3 + 0], b_xpos[curr_b * 3 + 1], b_xpos[curr_b * 3 + 2]);
        float3 r_j = p - anchor;
        float3 col_ax = cross(world_axis, r_j);

        J_p[0][dof_adr] = col_ax.x;
        J_p[1][dof_adr] = col_ax.y;
        J_p[2][dof_adr] = col_ax.z;

        curr_b = bodies[curr_b].parent_id;
    }
}

kernel void kernel_assemble_contact_constraints(
    device const float* contact_pos_batch [[buffer(0)]],        // (B, stride_nconmax, 3)
    device const float* contact_dist_batch [[buffer(1)]],       // (B, stride_nconmax)
    device const int* contact_body_batch [[buffer(2)]],         // (B, stride_nconmax)
    device const int* ncon_batch [[buffer(3)]],                 // (B,)
    device const float* body_xpos_batch [[buffer(4)]],          // (B, 17, 3)
    device const float* body_xmat_batch [[buffer(5)]],          // (B, 17, 9)
    device const float* qvel_batch [[buffer(6)]],              // (B, 20)
    constant BodyConstants* bodies [[buffer(7)]],               // 17 bodies
    device const float* body_invweight0 [[buffer(8)]],          // (17, 2)
    device const float* friction_batch [[buffer(9)]],           // (B, stride_nconmax, 2)
    device float* J_out [[buffer(10)]],                         // (B, stride_capacity, 20)
    device float* aref_out [[buffer(11)]],                      // (B, stride_capacity)
    device float* R_out [[buffer(12)]],                         // (B, stride_capacity)
    device int* efc_type_out [[buffer(13)]],                    // (B, stride_capacity)
    device int* nefc_out [[buffer(14)]],                        // (B,)
    device int* overflow_flag_out [[buffer(15)]],               // (B,)
    constant int& stride_nconmax [[buffer(16)]],                // contact input stride (e.g. 35)
    constant int& stride_capacity [[buffer(17)]],               // output allocation stride (e.g. 32/64)
    constant ContactSolverParams& params [[buffer(18)]],        // solver parameters
    constant int& active_capacity [[buffer(19)]],               // active capacity limit
    device const float* qpos_batch [[buffer(20)]],              // (B, 21) optional / for joint limits
    device const float* dof_frictionloss_batch [[buffer(21)]],  // (B, 14) optional / for dof frictionloss
    device float* frictionloss_out [[buffer(22)]],              // (B, stride_capacity) output loss bounds
    device int* efc_id_out [[buffer(23)]],                      // (B, stride_capacity) output constraint IDs
    device const float* dof_invweight0_batch [[buffer(24)]],    // (B, 14) or (14,) optional
    constant int& invweight_flags [[buffer(25)]],               // bit 0: batch body_invw, bit 1: batch dof_invw
    device const int* contact_body2_batch [[buffer(26)]],       // (B, stride_nconmax) optional second body ID
    device const float* contact_frame_batch [[buffer(27)]],     // (B, stride_nconmax, 9) optional 3x3 contact frame
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;

    // Kinematic state non-finite guard
    device const float* b_xpos = body_xpos_batch + b_idx * 17 * 3;
    device const float* b_xmat = body_xmat_batch + b_idx * 17 * 9;
    device const float* qvel = qvel_batch + b_idx * 20;

    bool finite_state = true;
    for (int i = 0; i < 17 * 3; ++i) { if (!isfinite(b_xpos[i])) { finite_state = false; break; } }
    for (int i = 0; i < 17 * 9; ++i) { if (!isfinite(b_xmat[i])) { finite_state = false; break; } }
    for (int i = 0; i < 20; ++i) { if (!isfinite(qvel[i])) { finite_state = false; break; } }
    if (qpos_batch != nullptr) {
        device const float* qp = qpos_batch + b_idx * 21;
        for (int i = 0; i < 21; ++i) { if (!isfinite(qp[i])) { finite_state = false; break; } }
    }
    if (dof_frictionloss_batch != nullptr) {
        device const float* fl = dof_frictionloss_batch + b_idx * 14;
        for (int i = 0; i < 14; ++i) { if (!isfinite(fl[i]) || fl[i] < 0.0f) { finite_state = false; break; } }
    }
    if (!finite_state) {
        nefc_out[b_idx] = 0;
        overflow_flag_out[b_idx] = -1; // Non-finite state error
        for (int r = 0; r < stride_capacity; ++r) {
            aref_out[b_idx * stride_capacity + r] = NAN;
            R_out[b_idx * stride_capacity + r] = NAN;
            efc_type_out[b_idx * stride_capacity + r] = -1;
            if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = NAN;
            if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = -1;
            for (int col = 0; col < 20; ++col) {
                J_out[b_idx * stride_capacity * 20 + r * 20 + col] = NAN;
            }
        }
        return;
    }

    int ncon = ncon_batch[b_idx];
    if (ncon < 0 || ncon > stride_nconmax) {
        nefc_out[b_idx] = 0;
        overflow_flag_out[b_idx] = -4; // Invalid contact count error
        for (int r = 0; r < stride_capacity; ++r) {
            aref_out[b_idx * stride_capacity + r] = NAN;
            R_out[b_idx * stride_capacity + r] = NAN;
            efc_type_out[b_idx * stride_capacity + r] = -1;
            if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = NAN;
            if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = -1;
            for (int col = 0; col < 20; ++col) {
                J_out[b_idx * stride_capacity * 20 + r * 20 + col] = NAN;
            }
        }
        return;
    }

    int rows_assembled = 0;
    int overflow = 0;
    int max_rows = min(stride_capacity, active_capacity);

    // 1. Assemble Contact Constraints
    if (ncon > 0) {
        int max_contact_rows = 4 * (max_rows / 4);
        int max_contacts = min(ncon, max_contact_rows / 4);
        if (ncon * 4 > max_contact_rows) {
            overflow = 1;
        }

        device const float* c_pos = contact_pos_batch + b_idx * stride_nconmax * 3;
        device const float* c_dist = contact_dist_batch + b_idx * stride_nconmax;
        device const int* c_body = contact_body_batch + b_idx * stride_nconmax;
        device const float* f_batch = friction_batch + b_idx * stride_nconmax * 2;
        device const int* c_body2 = (contact_body2_batch != nullptr) ? (contact_body2_batch + b_idx * stride_nconmax) : nullptr;
        device const float* c_frame = (contact_frame_batch != nullptr) ? (contact_frame_batch + b_idx * stride_nconmax * 9) : nullptr;

        float3 base_pos = float3(b_xpos[2 * 3 + 0], b_xpos[2 * 3 + 1], b_xpos[2 * 3 + 2]);
        float3x3 base_mat = load_row_major_mat3(b_xmat + 2 * 9);

        // MuJoCo spring-damper reference constants
        float K = 1.0f / (params.dmax * params.dmax * params.timeconst * params.timeconst * params.dampratio * params.dampratio);
        float B = 2.0f / (params.dmax * params.timeconst);

        for (int c = 0; c < max_contacts; ++c) {
            float3 p = float3(c_pos[c * 3 + 0], c_pos[c * 3 + 1], c_pos[c * 3 + 2]);
            float dist = c_dist[c];
            int b1_id = c_body[c];
            int b2_id = (c_body2 != nullptr) ? c_body2[c] : 0;

            // If body2 is ground (<=1) and body1 is a robot link (>1), swap to match MuJoCo convention (b1=ground, b2=foot)
            if (b2_id <= 1 && b1_id > 1) {
                int tmp = b1_id;
                b1_id = b2_id;
                b2_id = tmp;
            }

            // Validate body IDs and finiteness
            bool valid_bodies = (b1_id >= 0 && b1_id < 17 && b2_id >= 0 && b2_id < 17 && b1_id != b2_id);
            if (!valid_bodies || !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(dist)) {
                nefc_out[b_idx] = 0;
                overflow_flag_out[b_idx] = (!valid_bodies) ? -2 : -1;
                for (int r = 0; r < stride_capacity; ++r) {
                    aref_out[b_idx * stride_capacity + r] = NAN;
                    R_out[b_idx * stride_capacity + r] = NAN;
                    efc_type_out[b_idx * stride_capacity + r] = -1;
                    if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = NAN;
                    if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = -1;
                    for (int col = 0; col < 20; ++col) {
                        J_out[b_idx * stride_capacity * 20 + r * 20 + col] = NAN;
                    }
                }
                return;
            }

            // 1. Diagonal approximation from body inverse weights
            int body_invw_offset = (invweight_flags & 1) ? (b_idx * 17 * 2) : 0;
            float tran1 = (b1_id <= 1) ? 0.0f : body_invweight0[body_invw_offset + b1_id * 2 + 0];
            float tran2 = (b2_id <= 1) ? 0.0f : body_invweight0[body_invw_offset + b2_id * 2 + 0];
            float tran = tran1 + tran2;
            float mu1 = f_batch[c * 2 + 0];
            float mu2 = f_batch[c * 2 + 1];
            if (mu1 <= 0.0f || mu2 <= 0.0f || !isfinite(mu1) || !isfinite(mu2)) {
                nefc_out[b_idx] = 0;
                overflow_flag_out[b_idx] = -3; // Invalid or non-finite friction error
                for (int r = 0; r < stride_capacity; ++r) {
                    aref_out[b_idx * stride_capacity + r] = NAN;
                    R_out[b_idx * stride_capacity + r] = NAN;
                    efc_type_out[b_idx * stride_capacity + r] = -1;
                    if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = NAN;
                    if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = -1;
                    for (int col = 0; col < 20; ++col) {
                        J_out[b_idx * stride_capacity * 20 + r * 20 + col] = NAN;
                    }
                }
                return;
            }
            float mu0 = mu1;
            float dA0 = tran + (mu0 * mu0) * tran;

            // 2. Impedance spline
            float x = abs(dist - params.margin) / params.width;
            float imp = params.dmin;
            if (x <= 0.0f) {
                imp = params.dmin;
            } else if (x >= 1.0f) {
                imp = params.dmax;
            } else if (x < params.midpoint) {
                float a_imp = 1.0f / pow(params.midpoint, params.power - 1.0f);
                imp = params.dmin + a_imp * pow(x, params.power) * (params.dmax - params.dmin);
            } else {
                float b_imp = 1.0f / pow(1.0f - params.midpoint, params.power - 1.0f);
                imp = params.dmin + (1.0f - b_imp * pow(1.0f - x, params.power)) * (params.dmax - params.dmin);
            }

            // 3. Regularization R
            float R0 = (1.0f - imp) * dA0 / imp;
            float R1 = R0 / max(1e-14f, params.impratio);
            float mu_reg = mu0 * sqrt(R1 / R0);
            float Rpy = 2.0f * (mu_reg * mu_reg) * R0;

            // 4. Point Jacobians for b1 and b2
            float J_p1[3][20];
            float J_p2[3][20];
            compute_body_point_jacobian(b1_id, p, base_pos, base_mat, b_xpos, b_xmat, bodies, J_p1);
            compute_body_point_jacobian(b2_id, p, base_pos, base_mat, b_xpos, b_xmat, bodies, J_p2);

            float J_dif[3][20];
            for (int r = 0; r < 3; ++r) {
                for (int col = 0; col < 20; ++col) {
                    J_dif[r][col] = J_p2[r][col] - J_p1[r][col];
                }
            }

            // 5. Contact frame rotation
            float3 frame_rows[3];
            if (c_frame != nullptr) {
                frame_rows[0] = float3(c_frame[c * 9 + 0], c_frame[c * 9 + 1], c_frame[c * 9 + 2]);
                frame_rows[1] = float3(c_frame[c * 9 + 3], c_frame[c * 9 + 4], c_frame[c * 9 + 5]);
                frame_rows[2] = float3(c_frame[c * 9 + 6], c_frame[c * 9 + 7], c_frame[c * 9 + 8]);
            } else {
                // Default ground contact frame: normal +Z, tangent1 +Y, tangent2 -X
                frame_rows[0] = float3(0.0f, 0.0f, 1.0f);
                frame_rows[1] = float3(0.0f, 1.0f, 0.0f);
                frame_rows[2] = float3(-1.0f, 0.0f, 0.0f);
            }

            float J_rot[3][20];
            for (int r = 0; r < 3; ++r) {
                float3 f_row = frame_rows[r];
                for (int col = 0; col < 20; ++col) {
                    J_rot[r][col] = f_row.x * J_dif[0][col] + f_row.y * J_dif[1][col] + f_row.z * J_dif[2][col];
                }
            }

            // 6. Four pyramidal facet directions
            for (int k = 0; k < 4; ++k) {
                int row = rows_assembled++;
                float v_rel = 0.0f;
                for (int dof = 0; dof < 20; ++dof) {
                    float j_val;
                    if (k == 0) j_val = J_rot[0][dof] + mu1 * J_rot[1][dof];
                    else if (k == 1) j_val = J_rot[0][dof] - mu1 * J_rot[1][dof];
                    else if (k == 2) j_val = J_rot[0][dof] + mu2 * J_rot[2][dof];
                    else j_val = J_rot[0][dof] - mu2 * J_rot[2][dof];

                    J_out[b_idx * stride_capacity * 20 + row * 20 + dof] = j_val;
                    v_rel += j_val * qvel[dof];
                }

                float aref_val = -B * v_rel - K * imp * (dist - params.margin);
                aref_out[b_idx * stride_capacity + row] = aref_val;
                R_out[b_idx * stride_capacity + row] = Rpy;
                efc_type_out[b_idx * stride_capacity + row] = 6; // mjCNSTR_CONTACT_PYRAMIDAL
                if (frictionloss_out != nullptr) {
                    frictionloss_out[b_idx * stride_capacity + row] = 0.0f;
                }
                if (efc_id_out != nullptr) {
                    efc_id_out[b_idx * stride_capacity + row] = c;
                }
            }
        }
    }

    // 2. Assemble Joint Limit Constraints (mjCNSTR_LIMIT_JOINT = 3)
    if (qpos_batch != nullptr) {
        device const float* qpos = qpos_batch + b_idx * 21;
        for (int j = 0; j < 14; ++j) {
            int dof_idx = 6 + j;
            float q = qpos[7 + j];
            float v = qvel[dof_idx];
            float q_min = JNT_RANGE_MIN[j];
            float q_max = JNT_RANGE_MAX[j];
            float invw;
            if (dof_invweight0_batch != nullptr) {
                int dof_invw_offset = (invweight_flags & 2) ? (b_idx * 14) : 0;
                invw = dof_invweight0_batch[dof_invw_offset + j];
            } else {
                invw = DOF_INVWEIGHT0[j];
            }

            if (q < q_min) {
                if (rows_assembled < stride_capacity && rows_assembled < active_capacity) {
                    int r = rows_assembled++;
                    for (int col = 0; col < 20; ++col) {
                        J_out[b_idx * stride_capacity * 20 + r * 20 + col] = (col == dof_idx) ? 1.0f : 0.0f;
                    }
                    float pos = q - q_min;
                    aref_out[b_idx * stride_capacity + r] = -2631.578947f * pos - 105.263158f * v;
                    R_out[b_idx * stride_capacity + r] = 0.052631578947f * invw;
                    efc_type_out[b_idx * stride_capacity + r] = 3; // mjCNSTR_LIMIT_JOINT
                    if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = 0.0f;
                    if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = j + 1;
                } else {
                    overflow = 1;
                }
            } else if (q > q_max) {
                if (rows_assembled < stride_capacity && rows_assembled < active_capacity) {
                    int r = rows_assembled++;
                    for (int col = 0; col < 20; ++col) {
                        J_out[b_idx * stride_capacity * 20 + r * 20 + col] = (col == dof_idx) ? -1.0f : 0.0f;
                    }
                    float pos = q_max - q;
                    aref_out[b_idx * stride_capacity + r] = -2631.578947f * pos - 105.263158f * (-v);
                    R_out[b_idx * stride_capacity + r] = 0.052631578947f * invw;
                    efc_type_out[b_idx * stride_capacity + r] = 3; // mjCNSTR_LIMIT_JOINT
                    if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = 0.0f;
                    if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = j + 1;
                } else {
                    overflow = 1;
                }
            }
        }
    }

    // 3. Assemble Friction Loss Constraints (mjCNSTR_FRICTION_DOF = 1)
    if (dof_frictionloss_batch != nullptr) {
        device const float* dof_fl = dof_frictionloss_batch + b_idx * 14;
        for (int j = 0; j < 14; ++j) {
            float loss_val = dof_fl[j];
            if (loss_val > 0.0f) {
                int dof_idx = 6 + j;
                float v = qvel[dof_idx];
                float invw;
                if (dof_invweight0_batch != nullptr) {
                    int dof_invw_offset = (invweight_flags & 2) ? (b_idx * 14) : 0;
                    invw = dof_invweight0_batch[dof_invw_offset + j];
                } else {
                    invw = DOF_INVWEIGHT0[j];
                }

                if (rows_assembled < stride_capacity && rows_assembled < active_capacity) {
                    int r = rows_assembled++;
                    for (int col = 0; col < 20; ++col) {
                        J_out[b_idx * stride_capacity * 20 + r * 20 + col] = (col == dof_idx) ? 1.0f : 0.0f;
                    }
                    aref_out[b_idx * stride_capacity + r] = -200.020002f * v;
                    R_out[b_idx * stride_capacity + r] = 0.010101010101f * invw;
                    efc_type_out[b_idx * stride_capacity + r] = 1; // mjCNSTR_FRICTION_DOF
                    if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = loss_val;
                    if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = dof_idx;
                } else {
                    overflow = 1;
                }
            }
        }
    }

    nefc_out[b_idx] = rows_assembled;
    overflow_flag_out[b_idx] = overflow;

    // Zero out unused rows up to stride_capacity
    for (int r = rows_assembled; r < stride_capacity; ++r) {
        aref_out[b_idx * stride_capacity + r] = 0.0f;
        R_out[b_idx * stride_capacity + r] = 1.0f;
        efc_type_out[b_idx * stride_capacity + r] = 6;
        if (frictionloss_out != nullptr) frictionloss_out[b_idx * stride_capacity + r] = 0.0f;
        if (efc_id_out != nullptr) efc_id_out[b_idx * stride_capacity + r] = -1;
        for (int col = 0; col < 20; ++col) {
            J_out[b_idx * stride_capacity * 20 + r * 20 + col] = 0.0f;
        }
    }
}

// -----------------------------------------------------------------------------
// 2b. Contact Merging Kernel (Ground Contacts + Robot Self Contacts)
// -----------------------------------------------------------------------------

kernel void kernel_merge_contacts(
    device const float* ground_pos [[buffer(0)]],        // (B, stride_g, 3)
    device const float* ground_dist [[buffer(1)]],       // (B, stride_g)
    device const int* ground_body [[buffer(2)]],         // (B, stride_g)
    device const int* ground_ncon [[buffer(3)]],         // (B,)
    device const float* extra_pos [[buffer(4)]],         // (B, stride_e, 3)
    device const float* extra_dist [[buffer(5)]],        // (B, stride_e)
    device const int* extra_body1 [[buffer(6)]],         // (B, stride_e)
    device const int* extra_body2 [[buffer(7)]],         // (B, stride_e)
    device const float* extra_frame [[buffer(8)]],       // (B, stride_e, 9)
    device const float* extra_friction [[buffer(9)]],    // (B, stride_e, 2)
    device const int* extra_ncon [[buffer(10)]],         // (B,)
    device float* merged_pos [[buffer(11)]],             // (B, stride_m, 3)
    device float* merged_dist [[buffer(12)]],            // (B, stride_m)
    device int* merged_body1 [[buffer(13)]],             // (B, stride_m)
    device int* merged_body2 [[buffer(14)]],             // (B, stride_m)
    device float* merged_frame [[buffer(15)]],           // (B, stride_m, 9)
    device float* merged_friction [[buffer(16)]],        // (B, stride_m, 2)
    device int* merged_ncon [[buffer(17)]],              // (B,)
    device const float* ground_friction [[buffer(18)]],  // (B, 2) or (B, stride_g, 2)
    constant int& stride_g [[buffer(19)]],
    constant int& stride_e [[buffer(20)]],
    constant int& stride_m [[buffer(21)]],
    constant int& ground_friction_flag [[buffer(22)]],   // 0: default 0.8, 1: per_foot_friction (B, 2), 2: isotropic (B, 2), 3: per-contact (B, stride_g, 2)
    device const int* ground_geom [[buffer(23)]],        // (B, stride_g) optional for per_foot_friction
    device int* merged_overflow [[buffer(24)]],          // (B,) optional overflow count
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    int raw_g_in = (ground_ncon != nullptr) ? ground_ncon[b_idx] : 0;
    int raw_e_in = (extra_ncon != nullptr) ? extra_ncon[b_idx] : 0;
    if (raw_g_in < 0 || raw_e_in < 0) {
        if (merged_overflow != nullptr) merged_overflow[b_idx] = -4; // Invalid contact count
        merged_ncon[b_idx] = 0;
        for (int dst = 0; dst < stride_m; ++dst) {
            for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = NAN;
            merged_dist[b_idx * stride_m + dst] = NAN;
            merged_body1[b_idx * stride_m + dst] = -1;
            merged_body2[b_idx * stride_m + dst] = -1;
            for (int k = 0; k < 9; ++k) merged_frame[b_idx * stride_m * 9 + dst * 9 + k] = NAN;
            for (int k = 0; k < 2; ++k) merged_friction[b_idx * stride_m * 2 + dst * 2 + k] = NAN;
        }
        return;
    }

    int overflow_count = 0;
    if (raw_g_in > stride_g) overflow_count += (raw_g_in - stride_g);
    if (raw_e_in > stride_e) overflow_count += (raw_e_in - stride_e);
    if (raw_g_in + raw_e_in > stride_m) {
        int m_overflow = (raw_g_in + raw_e_in) - stride_m;
        if (m_overflow > overflow_count) overflow_count = m_overflow;
    }
    if (overflow_count > 0) {
        if (merged_overflow != nullptr) merged_overflow[b_idx] = overflow_count;
        merged_ncon[b_idx] = 0;
        for (int dst = 0; dst < stride_m; ++dst) {
            for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = NAN;
            merged_dist[b_idx * stride_m + dst] = NAN;
            merged_body1[b_idx * stride_m + dst] = -1;
            merged_body2[b_idx * stride_m + dst] = -1;
            for (int k = 0; k < 9; ++k) merged_frame[b_idx * stride_m * 9 + dst * 9 + k] = NAN;
            for (int k = 0; k < 2; ++k) merged_friction[b_idx * stride_m * 2 + dst * 2 + k] = NAN;
        }
        return;
    }
    if (merged_overflow != nullptr) {
        merged_overflow[b_idx] = 0;
    }

    int n_g = raw_g_in;
    int n_e = raw_e_in;

    // Non-finite input guard across active contacts
    bool finite_contacts = true;
    for (int i = 0; i < n_g; ++i) {
        for (int k = 0; k < 3; ++k) {
            if (!isfinite(ground_pos[b_idx * stride_g * 3 + i * 3 + k])) { finite_contacts = false; break; }
        }
        if (!isfinite(ground_dist[b_idx * stride_g + i])) { finite_contacts = false; break; }
    }
    if (finite_contacts) {
        for (int j = 0; j < n_e; ++j) {
            for (int k = 0; k < 3; ++k) {
                if (!isfinite(extra_pos[b_idx * stride_e * 3 + j * 3 + k])) { finite_contacts = false; break; }
            }
            if (!isfinite(extra_dist[b_idx * stride_e + j])) { finite_contacts = false; break; }
        }
    }

    if (!finite_contacts) {
        if (merged_overflow != nullptr) merged_overflow[b_idx] = -1; // Non-finite contact error
        merged_ncon[b_idx] = 0;
        for (int dst = 0; dst < stride_m; ++dst) {
            for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = NAN;
            merged_dist[b_idx * stride_m + dst] = NAN;
            merged_body1[b_idx * stride_m + dst] = -1;
            merged_body2[b_idx * stride_m + dst] = -1;
            for (int k = 0; k < 9; ++k) merged_frame[b_idx * stride_m * 9 + dst * 9 + k] = NAN;
            for (int k = 0; k < 2; ++k) merged_friction[b_idx * stride_m * 2 + dst * 2 + k] = NAN;
        }
        return;
    }

    int total_written = 0;

    // 1. Copy ground contacts
    for (int i = 0; i < n_g && total_written < stride_m; ++i) {
        int dst = total_written++;
        for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = ground_pos[b_idx * stride_g * 3 + i * 3 + k];
        merged_dist[b_idx * stride_m + dst] = ground_dist[b_idx * stride_g + i];
        merged_body1[b_idx * stride_m + dst] = ground_body[b_idx * stride_g + i];
        merged_body2[b_idx * stride_m + dst] = 0; // ground is body 0

        // Default ground frame: normal [0, 0, 1], tangent1 [0, 1, 0], tangent2 [-1, 0, 0]
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 0] = 0.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 1] = 0.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 2] = 1.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 3] = 0.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 4] = 1.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 5] = 0.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 6] = -1.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 7] = 0.0f;
        merged_frame[b_idx * stride_m * 9 + dst * 9 + 8] = 0.0f;

        float mu0 = 0.8f;
        float mu1 = 0.8f;
        if (ground_friction_flag == 1 && ground_friction != nullptr) {
            int g_id = (ground_geom != nullptr) ? ground_geom[b_idx * stride_g + i] : 1; // 1: left, 2: right
            int f_idx = (g_id == 2) ? 1 : 0;
            mu0 = ground_friction[b_idx * 2 + f_idx];
            mu1 = mu0;
        } else if (ground_friction_flag == 2 && ground_friction != nullptr) {
            mu0 = ground_friction[b_idx * 2 + 0];
            mu1 = ground_friction[b_idx * 2 + 1];
        } else if (ground_friction_flag == 3 && ground_friction != nullptr) {
            mu0 = ground_friction[b_idx * stride_g * 2 + i * 2 + 0];
            mu1 = ground_friction[b_idx * stride_g * 2 + i * 2 + 1];
        }
        merged_friction[b_idx * stride_m * 2 + dst * 2 + 0] = mu0;
        merged_friction[b_idx * stride_m * 2 + dst * 2 + 1] = mu1;
    }

    // 2. Copy extra (self) contacts
    for (int j = 0; j < n_e && total_written < stride_m; ++j) {
        int dst = total_written++;
        for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = extra_pos[b_idx * stride_e * 3 + j * 3 + k];
        merged_dist[b_idx * stride_m + dst] = extra_dist[b_idx * stride_e + j];
        merged_body1[b_idx * stride_m + dst] = (extra_body1 != nullptr) ? extra_body1[b_idx * stride_e + j] : 0;
        merged_body2[b_idx * stride_m + dst] = (extra_body2 != nullptr) ? extra_body2[b_idx * stride_e + j] : 0;

        if (extra_frame != nullptr) {
            for (int k = 0; k < 9; ++k) merged_frame[b_idx * stride_m * 9 + dst * 9 + k] = extra_frame[b_idx * stride_e * 9 + j * 9 + k];
        } else {
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 0] = 0.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 1] = 0.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 2] = 1.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 3] = 0.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 4] = 1.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 5] = 0.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 6] = -1.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 7] = 0.0f;
            merged_frame[b_idx * stride_m * 9 + dst * 9 + 8] = 0.0f;
        }

        if (extra_friction != nullptr) {
            merged_friction[b_idx * stride_m * 2 + dst * 2 + 0] = extra_friction[b_idx * stride_e * 2 + j * 2 + 0];
            merged_friction[b_idx * stride_m * 2 + dst * 2 + 1] = extra_friction[b_idx * stride_e * 2 + j * 2 + 1];
        } else {
            merged_friction[b_idx * stride_m * 2 + dst * 2 + 0] = 0.8f;
            merged_friction[b_idx * stride_m * 2 + dst * 2 + 1] = 0.8f;
        }
    }

    merged_ncon[b_idx] = total_written;

    // Zero out unused slots
    for (int dst = total_written; dst < stride_m; ++dst) {
        for (int k = 0; k < 3; ++k) merged_pos[b_idx * stride_m * 3 + dst * 3 + k] = 0.0f;
        merged_dist[b_idx * stride_m + dst] = 0.0f;
        merged_body1[b_idx * stride_m + dst] = 0;
        merged_body2[b_idx * stride_m + dst] = 0;
        for (int k = 0; k < 9; ++k) merged_frame[b_idx * stride_m * 9 + dst * 9 + k] = 0.0f;
        for (int k = 0; k < 2; ++k) merged_friction[b_idx * stride_m * 2 + dst * 2 + k] = 0.0f;
    }
}

// -----------------------------------------------------------------------------
// 3. Constrained Solve Kernel
// -----------------------------------------------------------------------------

kernel void kernel_constrained_solve(
    device const float* M_inv_batch [[buffer(0)]],          // (B, 20, 20)
    device const float* qfrc_bias_batch [[buffer(1)]],      // (B, 20)
    device const float* contact_pos_batch [[buffer(2)]],    // (B, nconmax, 3)
    device const float* contact_dist_batch [[buffer(3)]],   // (B, nconmax)
    device const int* contact_body_batch [[buffer(4)]],     // (B, nconmax)
    device const int* ncon_batch [[buffer(5)]],             // (B,)
    device const float* body_xpos_batch [[buffer(6)]],      // (B, 17, 3)
    device const float* body_xmat_batch [[buffer(7)]],      // (B, 17, 9) row-major
    constant BodyConstants* bodies [[buffer(8)]],           // 17 bodies
    constant float& friction_coef [[buffer(9)]],            // mu
    constant int& nconmax [[buffer(10)]],                   // capacity
    device const float* qvel_batch [[buffer(11)]],          // (B, 20)
    device float* qacc_out [[buffer(12)]],                  // (B, 20)
    device float* qfrc_constraint_out [[buffer(13)]],       // (B, 20)
    device const int* solver_status_batch [[buffer(14)]],   // (B,) optional / solver status from Cholesky
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device float* qacc = qacc_out + b_idx * 20;
    device float* qfrc_c = qfrc_constraint_out + b_idx * 20;

    // Propagate upstream factorization / solve failure into downstream outputs
    if (solver_status_batch != nullptr && solver_status_batch[b_idx] != 0) {
        for (int i = 0; i < 20; ++i) {
            qacc[i] = NAN;
            qfrc_c[i] = NAN;
        }
        return;
    }

    device const float* M_inv = M_inv_batch + b_idx * 400;

    // Guard against non-finite entries in M_inv
    for (int i = 0; i < 400; ++i) {
        if (!isfinite(M_inv[i])) {
            for (int k = 0; k < 20; ++k) {
                qacc[k] = NAN;
                qfrc_c[k] = NAN;
            }
            return;
        }
    }

    // Guard against non-finite entries in bias
    device const float* bias = qfrc_bias_batch + b_idx * 20;
    for (int i = 0; i < 20; ++i) {
        if (!isfinite(bias[i])) {
            for (int k = 0; k < 20; ++k) {
                qacc[k] = NAN;
                qfrc_c[k] = NAN;
            }
            return;
        }
    }

    int ncon = ncon_batch[b_idx];
    if (ncon <= 0) {
        for (int i = 0; i < 20; ++i) {
            qfrc_c[i] = 0.0f;
            float sum = 0.0f;
            for (int j = 0; j < 20; ++j) {
                sum += M_inv[i * 20 + j] * (-bias[j]);
            }
            qacc[i] = sum;
        }
        return;
    }

    int nefc = ncon * 4;
    if (nefc > 32) nefc = 32;

    device const float* c_pos = contact_pos_batch + b_idx * nconmax * 3;
    device const float* c_dist = contact_dist_batch + b_idx * nconmax;
    device const int* c_body = contact_body_batch + b_idx * nconmax;
    device const float* b_xpos = body_xpos_batch + b_idx * 17 * 3;
    device const float* b_xmat = body_xmat_batch + b_idx * 17 * 9;
    device const float* qvel = qvel_batch + b_idx * 20;

    // 1. Assemble contact Jacobian J (nefc x 20)
    float J[32][20];
    float aref[32];
    float D_diag[32];

    float3 base_pos = float3(b_xpos[2 * 3 + 0], b_xpos[2 * 3 + 1], b_xpos[2 * 3 + 2]);

    for (int c = 0; c < ncon && c * 4 < 32; ++c) {
        int efc_adr = c * 4;
        float3 p = float3(c_pos[c * 3 + 0], c_pos[c * 3 + 1], c_pos[c * 3 + 2]);
        float dist = c_dist[c];
        int body_id = c_body[c];

        float J_p[3][20];
        for (int r = 0; r < 3; ++r) {
            for (int col = 0; col < 20; ++col) {
                J_p[r][col] = 0.0f;
            }
        }

        // Freejoint linear DOFs (0, 1, 2)
        J_p[0][0] = 1.0f;
        J_p[1][1] = 1.0f;
        J_p[2][2] = 1.0f;

        // Freejoint angular DOFs (3, 4, 5): -r x
        float3 r = p - base_pos;
        J_p[0][3] = 0.0f;    J_p[0][4] = r.z;    J_p[0][5] = -r.y;
        J_p[1][3] = -r.z;   J_p[1][4] = 0.0f;   J_p[1][5] = r.x;
        J_p[2][3] = r.y;    J_p[2][4] = -r.x;   J_p[2][5] = 0.0f;

        // Ancestor hinge joints
        int curr_b = body_id;
        while (curr_b > 2) {
            int dof_adr = bodies[curr_b].dof_adr;
            float3 local_axis = float3(bodies[curr_b].jnt_axis);
            float3x3 mat_b = load_row_major_mat3(b_xmat + curr_b * 9);
            float3 world_axis = mat_b * local_axis;
            float3 anchor = float3(b_xpos[curr_b * 3 + 0], b_xpos[curr_b * 3 + 1], b_xpos[curr_b * 3 + 2]);
            float3 r_j = p - anchor;
            float3 col = cross(world_axis, r_j);

            J_p[0][dof_adr] = col.x;
            J_p[1][dof_adr] = col.y;
            J_p[2][dof_adr] = col.z;

            curr_b = bodies[curr_b].parent_id;
        }

        // Pyramidal cone rows (condim=3)
        // Normal = z, tangent1 = y, tangent2 = -x
        float mu = friction_coef;
        for (int col = 0; col < 20; ++col) {
            J[efc_adr + 0][col] = J_p[2][col] + mu * J_p[1][col];
            J[efc_adr + 1][col] = J_p[2][col] - mu * J_p[1][col];
            J[efc_adr + 2][col] = J_p[2][col] - mu * J_p[0][col];
            J[efc_adr + 3][col] = J_p[2][col] + mu * J_p[0][col];
        }

        float timeconst = 0.02f;
        float dampratio = 1.0f;
        float dmax = 0.95f;
        float width = 0.001f;

        float k = 1.0f / (dmax * dmax * timeconst * timeconst * dampratio * dampratio);
        float b_damp = 2.0f / (dmax * timeconst);
        float imp_x = abs(dist) / width;
        float imp = (imp_x > 1.0f) ? dmax : 0.9f;

        float invweight0 = 33.333333f;
        float invweight_pyr = (invweight0 + mu * mu * invweight0) * 2.0f * mu * mu;
        float D_val = 1.0f / (invweight_pyr * (1.0f - imp) / imp);

        for (int r = 0; r < 4; ++r) {
            float vel_r = 0.0f;
            for (int col = 0; col < 20; ++col) {
                vel_r += J[efc_adr + r][col] * qvel[col];
            }
            aref[efc_adr + r] = -k * imp * dist - b_damp * vel_r;
            D_diag[efc_adr + r] = D_val;
        }
    }

    // 2. Unconstrained acceleration qacc_0 = -M_inv * bias
    float qacc_0[20];
    for (int i = 0; i < 20; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < 20; ++j) {
            sum += M_inv[i * 20 + j] * (-bias[j]);
        }
        qacc_0[i] = sum;
    }

    // 3. Free constraint acceleration a_0 = J * qacc_0 - aref
    float a_0[32];
    for (int i = 0; i < nefc; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < 20; ++j) {
            sum += J[i][j] * qacc_0[j];
        }
        a_0[i] = sum - aref[i];
    }

    // 4. Delassus matrix A = J * M_inv * J^T + diag(1/D)
    float A[32][32];
    float J_Minv[32][20];

    for (int i = 0; i < nefc; ++i) {
        for (int j = 0; j < 20; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 20; ++k) {
                sum += J[i][k] * M_inv[k * 20 + j];
            }
            J_Minv[i][j] = sum;
        }
    }

    for (int i = 0; i < nefc; ++i) {
        for (int j = 0; j < nefc; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 20; ++k) {
                sum += J_Minv[i][k] * J[j][k];
            }
            if (i == j) {
                sum += 1.0f / D_diag[i];
            }
            A[i][j] = sum;
        }
    }

    // 5. Projected Gauss-Seidel (PGS) solve for lambda >= 0
    float lambda[32];
    for (int i = 0; i < nefc; ++i) {
        lambda[i] = 0.0f;
    }

    for (int iter = 0; iter < 100; ++iter) {
        for (int i = 0; i < nefc; ++i) {
            float row_dot = 0.0f;
            for (int j = 0; j < nefc; ++j) {
                row_dot += A[i][j] * lambda[j];
            }
            float delta = -(a_0[i] + row_dot) / A[i][i];
            lambda[i] = max(0.0f, lambda[i] + delta);
        }
    }

    // 6. Compute constraint force qfrc_constraint = J^T * lambda
    for (int j = 0; j < 20; ++j) {
        float sum = 0.0f;
        for (int i = 0; i < nefc; ++i) {
            sum += J[i][j] * lambda[i];
        }
        qfrc_c[j] = sum;
    }

    // 7. Solved acceleration qacc = qacc_0 + M_inv * qfrc_constraint
    for (int i = 0; i < 20; ++i) {
        float sum = qacc_0[i];
        for (int j = 0; j < 20; ++j) {
            sum += M_inv[i * 20 + j] * qfrc_c[j];
        }
        qacc[i] = sum;
    }
}

// -----------------------------------------------------------------------------
// 4. Native Cholesky Factorization and Multi-RHS Linear Solve Kernel
// -----------------------------------------------------------------------------

kernel void kernel_cholesky_solve(
    device const float* M_batch [[buffer(0)]],      // (B, 20, 20)
    device const float* B_batch [[buffer(1)]],      // (B, 20, K)
    constant int& K [[buffer(2)]],                  // number of RHS columns
    device float* L_out [[buffer(3)]],              // (B, 20, 20)
    device float* X_out [[buffer(4)]],              // (B, 20, K)
    device int* status_out [[buffer(5)]],           // (B,) 0: success, -1: non-positive pivot/NaN, -2: non-finite RHS, -3: solve failure
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device const float* M = M_batch + b_idx * 400;
    device const float* B = B_batch + b_idx * 20 * K;
    device float* L = L_out + b_idx * 400;
    device float* X = X_out + b_idx * 20 * K;

    // Check input M for non-finite entries
    for (int i = 0; i < 400; ++i) {
        if (!isfinite(M[i])) {
            status_out[b_idx] = -1;
            for (int k = 0; k < 400; ++k) L[k] = NAN;
            for (int k = 0; k < 20 * K; ++k) X[k] = NAN;
            return;
        }
    }

    // Check input B for non-finite entries
    for (int i = 0; i < 20 * K; ++i) {
        if (!isfinite(B[i])) {
            status_out[b_idx] = -2;
            for (int k = 0; k < 400; ++k) L[k] = NAN;
            for (int k = 0; k < 20 * K; ++k) X[k] = NAN;
            return;
        }
    }

    float L_loc[20][20];
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            L_loc[i][j] = 0.0f;
        }
    }

    status_out[b_idx] = 0;

    // Cholesky factorization: M = L * L^T
    for (int j = 0; j < 20; ++j) {
        float sum_sq = 0.0f;
        for (int k = 0; k < j; ++k) {
            sum_sq += L_loc[j][k] * L_loc[j][k];
        }
        float s = M[j * 20 + j] - sum_sq;
        if (s <= 0.0f || !isfinite(s)) {
            status_out[b_idx] = -1; // non-positive pivot or non-finite detected
            for (int k = 0; k < 400; ++k) L[k] = NAN;
            for (int k = 0; k < 20 * K; ++k) X[k] = NAN;
            return;
        }
        float diag = sqrt(s);
        L_loc[j][j] = diag;
        float inv_diag = 1.0f / diag;

        for (int i = j + 1; i < 20; ++i) {
            float sum_prod = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum_prod += L_loc[i][k] * L_loc[j][k];
            }
            L_loc[i][j] = (M[i * 20 + j] - sum_prod) * inv_diag;
        }
    }

    // Write factor L to device output
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            L[i * 20 + j] = L_loc[i][j];
        }
    }

    // Solve M X = B for each column c in [0, K-1]
    float Y_loc[20];
    for (int c = 0; c < K; ++c) {
        // Forward solve: L Y = B
        for (int i = 0; i < 20; ++i) {
            float s = B[i * K + c];
            for (int k = 0; k < i; ++k) {
                s -= L_loc[i][k] * Y_loc[k];
            }
            Y_loc[i] = s / L_loc[i][i];
        }

        // Back solve: L^T X = Y
        for (int i = 19; i >= 0; --i) {
            float s = Y_loc[i];
            for (int k = i + 1; k < 20; ++k) {
                s -= L_loc[k][i] * X[k * K + c];
            }
            float x_val = s / L_loc[i][i];
            if (!isfinite(x_val)) {
                status_out[b_idx] = -3;
                for (int k = 0; k < 20 * K; ++k) X[k] = NAN;
                return;
            }
            X[i * K + c] = x_val;
        }
    }
}

// -----------------------------------------------------------------------------
// 5. Oracle-Supplied Pyramidal Contact Constrained Solve Kernel
// -----------------------------------------------------------------------------

constant int MAX_ORACLE_CONSTRAINTS = 128;

kernel void kernel_oracle_constrained_solve(
    device const float* L_batch [[buffer(0)]],                // (B, 20, 20) Cholesky factor M = L L^T
    device const float* f_smooth_batch [[buffer(1)]],         // (B, 20) Complete smooth generalized forces
    device const float* J_batch [[buffer(2)]],                // (B, capacity, 20) Constraint Jacobian
    device const float* aref_batch [[buffer(3)]],             // (B, capacity) Reference acceleration
    device const float* R_batch [[buffer(4)]],                // (B, capacity) Constraint regularization (efc_R)
    device const int* efc_type_batch [[buffer(5)]],           // (B, capacity) Constraint row types (6, 3, 1)
    device const int* nefc_batch [[buffer(6)]],               // (B,) Number of active constraint rows
    device const int* upstream_status_batch [[buffer(7)]],    // (B,) Upstream status (e.g. from Cholesky)
    device float* lambda_out [[buffer(8)]],                   // (B, capacity) Constraint forces
    device float* qfrc_constraint_out [[buffer(9)]],          // (B, 20) Generalized constraint forces J^T lambda
    device float* qacc_out [[buffer(10)]],                    // (B, 20) Final acceleration
    device int* solver_status_out [[buffer(11)]],             // (B,) Solver status (0: success, 1: unconverged, negative: error)
    device int* actual_iters_out [[buffer(12)]],              // (B,) Actual PGS iterations executed
    device float* dual_residual_out [[buffer(13)]],           // (B,) Max complementarity residual
    device float* scratchpad_A [[buffer(14)]],                // (B, capacity, capacity) preallocated Delassus matrix
    device float* scratchpad_Y [[buffer(15)]],                // (B, capacity, 20) preallocated factor projection
    device const float* frictionloss_batch [[buffer(16)]],    // (B, capacity) friction budget bounds for type 1
    constant int& max_iters [[buffer(17)]],                   // Maximum PGS iterations
    constant int& capacity [[buffer(18)]],                    // Buffer capacity (max supported is 128)
    constant float& tol [[buffer(19)]],                       // Convergence tolerance
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;
    device float* qacc = qacc_out + b_idx * 20;
    device float* qfrc_c = qfrc_constraint_out + b_idx * 20;
    device float* lam_out = lambda_out + b_idx * capacity;

    // 1. Upstream status propagation
    if (upstream_status_batch != nullptr && upstream_status_batch[b_idx] != 0) {
        solver_status_out[b_idx] = upstream_status_batch[b_idx];
        actual_iters_out[b_idx] = 0;
        dual_residual_out[b_idx] = NAN;
        for (int k = 0; k < 20; ++k) {
            qacc[k] = NAN;
            qfrc_c[k] = NAN;
        }
        for (int k = 0; k < capacity; ++k) {
            lam_out[k] = NAN;
        }
        return;
    }

    // 2. Capacity and nefc bounds guard
    int nefc = nefc_batch[b_idx];
    if (nefc < 0 || nefc > capacity || nefc > MAX_ORACLE_CONSTRAINTS) {
        solver_status_out[b_idx] = -4; // Capacity overflow or invalid nefc
        actual_iters_out[b_idx] = 0;
        dual_residual_out[b_idx] = NAN;
        for (int k = 0; k < 20; ++k) {
            qacc[k] = NAN;
            qfrc_c[k] = NAN;
        }
        for (int k = 0; k < capacity; ++k) {
            lam_out[k] = NAN;
        }
        return;
    }

    // 3. Supported constraint types guard (accept 6: pyramidal contact, 3: joint limit, 1: friction loss)
    device const int* efc_type = efc_type_batch + b_idx * capacity;
    for (int i = 0; i < nefc; ++i) {
        int t = efc_type[i];
        if (t != 6 && t != 3 && t != 1) {
            solver_status_out[b_idx] = -5; // Unsupported constraint row type
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) {
                qacc[k] = NAN;
                qfrc_c[k] = NAN;
            }
            for (int k = 0; k < capacity; ++k) {
                lam_out[k] = NAN;
            }
            return;
        }
    }

    // 4. Non-finite input guards
    device const float* L = L_batch + b_idx * 400;
    device const float* f_smooth = f_smooth_batch + b_idx * 20;
    device const float* J = J_batch + b_idx * capacity * 20;
    device const float* aref = aref_batch + b_idx * capacity;
    device const float* R = R_batch + b_idx * capacity;

    for (int i = 0; i < 400; ++i) {
        if (!isfinite(L[i])) {
            solver_status_out[b_idx] = -1;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
    }
    for (int i = 0; i < 20; ++i) {
        if (!isfinite(f_smooth[i]) || L[i * 20 + i] <= 0.0f) {
            solver_status_out[b_idx] = -1;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
    }
    for (int i = 0; i < nefc; ++i) {
        if (!isfinite(aref[i]) || !isfinite(R[i]) || R[i] <= 0.0f) {
            solver_status_out[b_idx] = -1;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        for (int k = 0; k < 20; ++k) {
            if (!isfinite(J[i * 20 + k])) {
                solver_status_out[b_idx] = -1;
                actual_iters_out[b_idx] = 0;
                dual_residual_out[b_idx] = NAN;
                for (int p = 0; p < 20; ++p) { qacc[p] = NAN; qfrc_c[p] = NAN; }
                for (int p = 0; p < capacity; ++p) lam_out[p] = NAN;
                return;
            }
        }
    }

    // 5. Unconstrained acceleration solve: L L^T a_0 = f_smooth
    float y_0[20];
    for (int i = 0; i < 20; ++i) {
        float s = f_smooth[i];
        for (int p = 0; p < i; ++p) {
            s -= L[i * 20 + p] * y_0[p];
        }
        float y_val = s / L[i * 20 + i];
        if (!isfinite(y_val)) {
            solver_status_out[b_idx] = -3; // Solve overflow / non-finite acceleration
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        y_0[i] = y_val;
    }

    float a_0[20];
    for (int i = 19; i >= 0; --i) {
        float s = y_0[i];
        for (int p = i + 1; p < 20; ++p) {
            s -= L[p * 20 + i] * a_0[p];
        }
        float a_val = s / L[i * 20 + i];
        if (!isfinite(a_val)) {
            solver_status_out[b_idx] = -3; // Solve overflow / non-finite acceleration
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        a_0[i] = a_val;
    }

    // 6. Zero-contact shortcut (airborne / no constraints)
    if (nefc == 0) {
        for (int i = 0; i < 20; ++i) {
            qacc[i] = a_0[i];
            qfrc_c[i] = 0.0f;
        }
        for (int i = 0; i < capacity; ++i) {
            lam_out[i] = 0.0f;
        }
        solver_status_out[b_idx] = 0;
        actual_iters_out[b_idx] = 0;
        dual_residual_out[b_idx] = 0.0f;
        return;
    }

    // 7. Assemble Delassus matrix: L Y = J^T => Y = L^{-1} J^T, A = Y^T Y + diag(R) using device scratchpad
    device float* Y = scratchpad_Y + b_idx * capacity * 20;
    for (int i = 0; i < nefc; ++i) {
        for (int k = 0; k < 20; ++k) {
            float s = J[i * 20 + k];
            for (int p = 0; p < k; ++p) {
                s -= L[k * 20 + p] * Y[i * 20 + p];
            }
            float y_val = s / L[k * 20 + k];
            if (!isfinite(y_val)) {
                solver_status_out[b_idx] = -3;
                actual_iters_out[b_idx] = 0;
                dual_residual_out[b_idx] = NAN;
                for (int p_idx = 0; p_idx < 20; ++p_idx) { qacc[p_idx] = NAN; qfrc_c[p_idx] = NAN; }
                for (int p_idx = 0; p_idx < capacity; ++p_idx) lam_out[p_idx] = NAN;
                return;
            }
            Y[i * 20 + k] = y_val;
        }
    }

    device float* A = scratchpad_A + b_idx * capacity * capacity;
    for (int i = 0; i < nefc; ++i) {
        for (int j = 0; j < nefc; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 20; ++k) {
                s += Y[i * 20 + k] * Y[j * 20 + k];
            }
            if (i == j) {
                s += R[i];
            }
            A[i * capacity + j] = s;
        }
        if (A[i * capacity + i] <= 1e-12f || !isfinite(A[i * capacity + i])) {
            solver_status_out[b_idx] = -2; // Non-positive / singular Delassus diagonal
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
    }

    // 8. Assemble free constraint acceleration b = J a_0 - aref
    float b_vec[MAX_ORACLE_CONSTRAINTS];
    float lam[MAX_ORACLE_CONSTRAINTS];
    float g[MAX_ORACLE_CONSTRAINTS];
    for (int i = 0; i < nefc; ++i) {
        float s = 0.0f;
        for (int k = 0; k < 20; ++k) {
            s += J[i * 20 + k] * a_0[k];
        }
        float b_val = s - aref[i];
        if (!isfinite(b_val)) {
            solver_status_out[b_idx] = -3;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        b_vec[i] = b_val;
        lam[i] = 0.0f; // Deterministic zero-start
        g[i] = b_val;
    }

    // 9. Projected Gauss-Seidel (PGS) solve with bilateral friction clamping & unilateral projection
    int iters_taken = max_iters;
    device const float* fl_batch = (frictionloss_batch != nullptr) ? (frictionloss_batch + b_idx * capacity) : nullptr;

    for (int iter = 0; iter < max_iters; ++iter) {
        float max_delta = 0.0f;
        for (int i = 0; i < nefc; ++i) {
            float delta = -g[i] / A[i * capacity + i];
            float lam_new = lam[i] + delta;
            int t = efc_type[i];
            if (t == 6 || t == 3) {
                lam_new = max(0.0f, lam_new);
            } else if (t == 1) {
                float loss_i = (fl_batch != nullptr) ? fl_batch[i] : 0.0f;
                lam_new = clamp(lam_new, -loss_i, loss_i);
            }
            float d_act = lam_new - lam[i];
            if (abs(d_act) > 1e-12f) {
                for (int j = 0; j < nefc; ++j) {
                    g[j] += A[j * capacity + i] * d_act;
                }
                lam[i] = lam_new;
                max_delta = max(max_delta, abs(d_act));
            }
        }
        float cur_dual_infeas = 0.0f;
        for (int i = 0; i < nefc; ++i) {
            int t = efc_type[i];
            if (t == 6 || t == 3) {
                cur_dual_infeas = max(cur_dual_infeas, max(0.0f, -g[i]));
            } else if (t == 1) {
                float loss_i = (fl_batch != nullptr) ? fl_batch[i] : 0.0f;
                if (lam[i] >= loss_i - 1e-5f) {
                    cur_dual_infeas = max(cur_dual_infeas, max(0.0f, g[i]));
                } else if (lam[i] <= -loss_i + 1e-5f) {
                    cur_dual_infeas = max(cur_dual_infeas, max(0.0f, -g[i]));
                } else {
                    cur_dual_infeas = max(cur_dual_infeas, abs(g[i]));
                }
            }
        }
        if (max_delta < tol && cur_dual_infeas <= 1e-4f) {
            iters_taken = iter + 1;
            break;
        }
        if (max_delta < 1e-12f) {
            iters_taken = iter + 1;
            break;
        }
    }

    // 10. Diagonally scaled projected-gradient residual and dual infeasibility check
    float max_proj_res = 0.0f;
    float max_dual_infeas = 0.0f;
    for (int i = 0; i < nefc; ++i) {
        if (!isfinite(lam[i]) || !isfinite(g[i])) {
            solver_status_out[b_idx] = -3;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        int t = efc_type[i];
        if (t == 6 || t == 3) {
            max_dual_infeas = max(max_dual_infeas, max(0.0f, -g[i]));
            float step = abs(lam[i] - max(0.0f, lam[i] - g[i] / A[i * capacity + i]));
            max_proj_res = max(max_proj_res, step);
        } else if (t == 1) {
            float loss_i = (fl_batch != nullptr) ? fl_batch[i] : 0.0f;
            float step = abs(lam[i] - clamp(lam[i] - g[i] / A[i * capacity + i], -loss_i, loss_i));
            max_proj_res = max(max_proj_res, step);
            if (lam[i] >= loss_i - 1e-5f) {
                max_dual_infeas = max(max_dual_infeas, max(0.0f, g[i]));
            } else if (lam[i] <= -loss_i + 1e-5f) {
                max_dual_infeas = max(max_dual_infeas, max(0.0f, -g[i]));
            } else {
                max_dual_infeas = max(max_dual_infeas, abs(g[i]));
            }
        }
    }

    // 11. Generalized constraint forces: f_c = J^T * lambda
    for (int k = 0; k < 20; ++k) {
        float s = 0.0f;
        for (int i = 0; i < nefc; ++i) {
            s += J[i * 20 + k] * lam[i];
        }
        qfrc_c[k] = s;
    }

    // 12. Corrected acceleration: y_c = Y lambda, L^T delta_a = y_c, qacc = a_0 + delta_a
    float y_c[20];
    for (int k = 0; k < 20; ++k) {
        float s = 0.0f;
        for (int i = 0; i < nefc; ++i) {
            s += Y[i * 20 + k] * lam[i];
        }
        y_c[k] = s;
    }

    float delta_a[20];
    for (int i = 19; i >= 0; --i) {
        float s = y_c[i];
        for (int p = i + 1; p < 20; ++p) {
            s -= L[p * 20 + i] * delta_a[p];
        }
        float da_val = s / L[i * 20 + i];
        delta_a[i] = da_val;
    }

    for (int i = 0; i < 20; ++i) {
        float qacc_val = a_0[i] + delta_a[i];
        if (!isfinite(qacc_val) || !isfinite(qfrc_c[i])) {
            solver_status_out[b_idx] = -3;
            actual_iters_out[b_idx] = 0;
            dual_residual_out[b_idx] = NAN;
            for (int k = 0; k < 20; ++k) { qacc[k] = NAN; qfrc_c[k] = NAN; }
            for (int k = 0; k < capacity; ++k) lam_out[k] = NAN;
            return;
        }
        qacc[i] = qacc_val;
    }

    // 13. Write output forces and clean inactive padding
    for (int i = 0; i < nefc; ++i) {
        lam_out[i] = lam[i];
    }
    for (int i = nefc; i < capacity; ++i) {
        lam_out[i] = 0.0f;
    }

    // Status: 0 if converged (projected gradient residual < tol and dual infeasibility <= 1e-4),
    //         1 if unconverged within max_iters (bounded physical state),
    //        -3 if non-finite.
    int final_status = (max_proj_res < tol && max_dual_infeas <= 1e-4f) ? 0 : 1;
    solver_status_out[b_idx] = final_status;
    actual_iters_out[b_idx] = iters_taken;
    dual_residual_out[b_idx] = max_proj_res;
}

// ==============================================================================
// 6. ImplicitFast Time-Integration Kernel
// ==============================================================================
kernel void kernel_integrate_implicit_fast(
    device const float* qpos_in [[buffer(0)]],            // (B, 21) current generalized coordinates
    device const float* qvel_in [[buffer(1)]],            // (B, 20) current generalized velocities
    device const float* qacc_in [[buffer(2)]],            // (B, 20) acceleration from solve
    device const int* upstream_status [[buffer(3)]],      // (B,) upstream solver / physics status
    constant float& dt [[buffer(4)]],                     // timestep (e.g. 0.005f)
    device float* qpos_out [[buffer(5)]],                 // (B, 21) advanced coordinates
    device float* qvel_out [[buffer(6)]],                 // (B, 20) advanced velocities
    device int* integration_status_out [[buffer(7)]],     // (B,) integration status
    device const float* M_batch [[buffer(8)]],            // (B, 20, 20) optional physical mass matrix
    device const float* damping_batch [[buffer(9)]],      // (B, 20) optional damping
    constant int& has_damping [[buffer(10)]],             // 1 if damping active, 0 otherwise
    uint tid [[thread_position_in_grid]]
) {
    uint b_idx = tid;

    // 1. Upstream status guard: negative status prevents integration of invalid accelerations
    int status = upstream_status[b_idx];
    if (status < 0) {
        integration_status_out[b_idx] = status;
        for (int i = 0; i < 21; ++i) {
            qpos_out[b_idx * 21 + i] = NAN;
        }
        for (int i = 0; i < 20; ++i) {
            qvel_out[b_idx * 20 + i] = NAN;
        }
        return;
    }

    device const float* qp = qpos_in + b_idx * 21;
    device const float* qv = qvel_in + b_idx * 20;
    device const float* qa = qacc_in + b_idx * 20;

    // 2. Non-finite input check
    bool finite_inputs = true;
    for (int i = 0; i < 21; ++i) {
        if (!isfinite(qp[i])) { finite_inputs = false; break; }
    }
    for (int i = 0; i < 20; ++i) {
        if (!isfinite(qv[i]) || !isfinite(qa[i])) { finite_inputs = false; break; }
    }
    if (!finite_inputs || !isfinite(dt) || dt <= 0.0f) {
        integration_status_out[b_idx] = -1; // Non-finite input error
        for (int i = 0; i < 21; ++i) {
            qpos_out[b_idx * 21 + i] = NAN;
        }
        for (int i = 0; i < 20; ++i) {
            qvel_out[b_idx * 20 + i] = NAN;
        }
        return;
    }

    // Current orientation quaternion (w, x, y, z)
    float w1 = qp[3], x1 = qp[4], y1 = qp[5], z1 = qp[6];
    float q1_sq = w1 * w1 + x1 * x1 + y1 * y1 + z1 * z1;
    if (q1_sq < 1e-8f || q1_sq > 1e8f || !isfinite(q1_sq)) {
        integration_status_out[b_idx] = -3; // Degenerate orientation input error
        for (int i = 0; i < 21; ++i) { qpos_out[b_idx * 21 + i] = NAN; }
        for (int i = 0; i < 20; ++i) { qvel_out[b_idx * 20 + i] = NAN; }
        return;
    }

    // 3. Velocity update:
    // If viscous damping is active: solve (M + dt * D) delta_v = dt * (M * qa)
    // Otherwise: delta_v = dt * qa
    float v_next[20];
    if (has_damping != 0 && damping_batch != nullptr && M_batch != nullptr) {
        device const float* M = M_batch + b_idx * 400;
        device const float* damp = damping_batch + b_idx * 20;

        float rhs[20];
        for (int i = 0; i < 20; ++i) {
            float s = 0.0f;
            for (int j = 0; j < 20; ++j) {
                s += M[i * 20 + j] * qa[j];
            }
            rhs[i] = dt * s;
        }

        float M_damp[20][20];
        for (int i = 0; i < 20; ++i) {
            for (int j = 0; j < 20; ++j) {
                M_damp[i][j] = M[i * 20 + j];
            }
            M_damp[i][i] += dt * damp[i];
        }

        // Cholesky factorization: M_damp = L * L^T
        float L[20][20];
        for (int i = 0; i < 20; ++i) {
            for (int j = 0; j < 20; ++j) {
                L[i][j] = 0.0f;
            }
        }

        bool chol_ok = true;
        for (int j = 0; j < 20; ++j) {
            float sum_sq = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum_sq += L[j][k] * L[j][k];
            }
            float s = M_damp[j][j] - sum_sq;
            if (s <= 0.0f || !isfinite(s)) {
                chol_ok = false;
                break;
            }
            float diag = sqrt(s);
            L[j][j] = diag;
            float inv_diag = 1.0f / diag;
            for (int i = j + 1; i < 20; ++i) {
                float sum_prod = 0.0f;
                for (int k = 0; k < j; ++k) {
                    sum_prod += L[i][k] * L[j][k];
                }
                L[i][j] = (M_damp[i][j] - sum_prod) * inv_diag;
            }
        }

        if (!chol_ok) {
            integration_status_out[b_idx] = -4; // Cholesky failure in implicit velocity solve
            for (int i = 0; i < 21; ++i) { qpos_out[b_idx * 21 + i] = NAN; }
            for (int i = 0; i < 20; ++i) { qvel_out[b_idx * 20 + i] = NAN; }
            return;
        }

        // Forward solve: L * Y = rhs
        float Y[20];
        for (int i = 0; i < 20; ++i) {
            float s = rhs[i];
            for (int k = 0; k < i; ++k) {
                s -= L[i][k] * Y[k];
            }
            Y[i] = s / L[i][i];
        }

        // Backward solve: L^T * delta_v = Y
        float delta_v[20];
        for (int i = 19; i >= 0; --i) {
            float s = Y[i];
            for (int k = i + 1; k < 20; ++k) {
                s -= L[k][i] * delta_v[k];
            }
            delta_v[i] = s / L[i][i];
        }

        for (int i = 0; i < 20; ++i) {
            v_next[i] = qv[i] + delta_v[i];
        }
    } else {
        for (int i = 0; i < 20; ++i) {
            v_next[i] = qv[i] + dt * qa[i];
        }
    }

    // 4. Position update:
    // Root linear translation (DOFs 0, 1, 2 -> qpos 0, 1, 2)
    float p_next[3];
    p_next[0] = qp[0] + dt * v_next[0];
    p_next[1] = qp[1] + dt * v_next[1];
    p_next[2] = qp[2] + dt * v_next[2];

    // Root orientation: Hamilton quaternion integration q_{t+h} = q_t * dq(omega_{t+h} * dt)
    // DOFs 3, 4, 5 represent angular velocity omega in local body frame
    float3 omega = float3(v_next[3], v_next[4], v_next[5]);
    float angle = length(omega) * dt;
    float w2, x2, y2, z2;
    if (angle > 1e-12f) {
        float3 axis = omega / length(omega);
        float s = sin(angle * 0.5f);
        w2 = cos(angle * 0.5f);
        x2 = s * axis.x;
        y2 = s * axis.y;
        z2 = s * axis.z;
    } else {
        w2 = 1.0f;
        x2 = 0.5f * dt * omega.x;
        y2 = 0.5f * dt * omega.y;
        z2 = 0.5f * dt * omega.z;
    }

    // Hamilton product: q_new = q_curr * dq
    float w_new = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    float x_new = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    float y_new = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    float z_new = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

    // Hinge joints (joints 1 to 14, DOFs 6..19 -> qpos 7..20)
    float jnt_next[14];
    for (int j = 0; j < 14; ++j) {
        jnt_next[j] = qp[7 + j] + dt * v_next[6 + j];
    }

    // 5. Post-arithmetic validation: detect overflow or non-finite computed coordinates/velocities
    bool finite_computed = true;
    for (int i = 0; i < 20; ++i) {
        if (!isfinite(v_next[i])) { finite_computed = false; break; }
    }
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(p_next[i])) { finite_computed = false; break; }
    }
    for (int j = 0; j < 14; ++j) {
        if (!isfinite(jnt_next[j])) { finite_computed = false; break; }
    }
    if (!finite_computed) {
        integration_status_out[b_idx] = -2; // Arithmetic overflow in state advancement
        for (int i = 0; i < 21; ++i) { qpos_out[b_idx * 21 + i] = NAN; }
        for (int i = 0; i < 20; ++i) { qvel_out[b_idx * 20 + i] = NAN; }
        return;
    }

    float q_new_sq = w_new * w_new + x_new * x_new + y_new * y_new + z_new * z_new;
    if (q_new_sq < 1e-8f || !isfinite(q_new_sq)) {
        integration_status_out[b_idx] = -3; // Degenerate orientation post-integration
        for (int i = 0; i < 21; ++i) { qpos_out[b_idx * 21 + i] = NAN; }
        for (int i = 0; i < 20; ++i) { qvel_out[b_idx * 20 + i] = NAN; }
        return;
    }

    float q_invnorm = rsqrt(q_new_sq);
    w_new *= q_invnorm;
    x_new *= q_invnorm;
    y_new *= q_invnorm;
    z_new *= q_invnorm;

    // 6. Write outputs (safe for in-place update where qpos_out == qpos_in or qvel_out == qvel_in)
    device float* qp_out = qpos_out + b_idx * 21;
    device float* qv_out = qvel_out + b_idx * 20;

    qp_out[0] = p_next[0];
    qp_out[1] = p_next[1];
    qp_out[2] = p_next[2];

    qp_out[3] = w_new;
    qp_out[4] = x_new;
    qp_out[5] = y_new;
    qp_out[6] = z_new;

    for (int j = 0; j < 14; ++j) {
        qp_out[7 + j] = jnt_next[j];
    }

    for (int i = 0; i < 20; ++i) {
        qv_out[i] = v_next[i];
    }

    integration_status_out[b_idx] = 0;
}
