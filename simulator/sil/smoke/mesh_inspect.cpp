/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 *
 * Throwaway tool: print each mesh geom's body-frame Z extent, to measure the
 * propeller-vs-motor-top gap so the props can be nudged to sit on the motors.
 * 使い捨てツール: 各メッシュ geom の機体系 Z 範囲を表示し、プロペラとモータ
 * 上面の隙間を測る。
 */

#include <cfloat>
#include <cstdio>
#include <mujoco/mujoco.h>

int main(int argc, char** argv)
{
    const char* path = (argc > 1) ? argv[1] : "simulator/sil/models/stampfly.xml";
    char err[1024] = {0};
    mjModel* m = mj_loadXML(path, nullptr, err, sizeof(err));
    if (!m) { fprintf(stderr, "load failed: %s\n", err); return 1; }

    printf("%-14s  z_min     z_max    (body frame, m)\n", "mesh");
    for (int g = 0; g < m->ngeom; ++g) {
        if (m->geom_type[g] != mjGEOM_MESH) continue;
        int mesh = m->geom_dataid[g];
        const char* name = mj_id2name(m, mjOBJ_MESH, mesh);

        double zmin = DBL_MAX, zmax = -DBL_MAX;
        int adr = m->mesh_vertadr[mesh];
        int n = m->mesh_vertnum[mesh];
        for (int k = 0; k < n; ++k) {
            mjtNum v[3] = {m->mesh_vert[3 * (adr + k) + 0],
                           m->mesh_vert[3 * (adr + k) + 1],
                           m->mesh_vert[3 * (adr + k) + 2]};
            mjtNum vr[3];
            mju_rotVecQuat(vr, v, m->geom_quat + 4 * g);   // mesh -> body rotation
            double zb = vr[2] + m->geom_pos[3 * g + 2];     // + geom pos (body Z)
            if (zb < zmin) zmin = zb;
            if (zb > zmax) zmax = zb;
        }
        printf("%-14s  %+.4f  %+.4f\n", name ? name : "?", zmin, zmax);

        // For motors: radius profile vs Z to find the CAN top (where the wide
        // can transitions to the thin shaft). 缶→軸で細くなる Z を探す。
        if (name && std::string(name).rfind("m_motor", 0) == 0) {
            double cx = 0, cy = 0;
            for (int k = 0; k < n; ++k) {
                mjtNum v[3] = {m->mesh_vert[3*(adr+k)+0], m->mesh_vert[3*(adr+k)+1], m->mesh_vert[3*(adr+k)+2]};
                mjtNum vr[3]; mju_rotVecQuat(vr, v, m->geom_quat + 4*g);
                cx += vr[0] + m->geom_pos[3*g+0]; cy += vr[1] + m->geom_pos[3*g+1];
            }
            cx /= n; cy /= n;
            const int NB = 12;
            double rmax[NB] = {0};
            for (int k = 0; k < n; ++k) {
                mjtNum v[3] = {m->mesh_vert[3*(adr+k)+0], m->mesh_vert[3*(adr+k)+1], m->mesh_vert[3*(adr+k)+2]};
                mjtNum vr[3]; mju_rotVecQuat(vr, v, m->geom_quat + 4*g);
                double xb=vr[0]+m->geom_pos[3*g+0], yb=vr[1]+m->geom_pos[3*g+1], zb=vr[2]+m->geom_pos[3*g+2];
                int b = (int)((zb - zmin) / (zmax - zmin) * (NB - 1) + 0.5);
                if (b < 0) b = 0; if (b >= NB) b = NB - 1;
                double r = std::sqrt((xb-cx)*(xb-cx)+(yb-cy)*(yb-cy));
                if (r > rmax[b]) rmax[b] = r;
            }
            printf("    -> radius profile (z low->high):");
            for (int b = 0; b < NB; ++b) printf(" %.0f", rmax[b]*1000.0);  // mm
            printf("  [mm]; z step=%.4f\n", (zmax - zmin) / (NB - 1));
            for (int b = 0; b < NB; ++b)
                printf("       band %2d  z=%.4f  rmax=%.4f m\n", b, zmin + b*(zmax-zmin)/(NB-1), rmax[b]);
        }

        // For propellers: is the small-radius hub at the bottom (on motor) or
        // the top (upside down)? Compare XY radius near z_min vs near z_max.
        // プロペラ: 細いハブが下(モータ側)か上(裏返し)か。z_min 付近と z_max
        // 付近の XY 半径を比べる。
        if (name && std::string(name).rfind("m_prop", 0) == 0) {
            // XY centroid (in body frame).
            double cx = 0, cy = 0;
            for (int k = 0; k < n; ++k) {
                mjtNum v[3] = {m->mesh_vert[3*(adr+k)+0], m->mesh_vert[3*(adr+k)+1], m->mesh_vert[3*(adr+k)+2]};
                mjtNum vr[3]; mju_rotVecQuat(vr, v, m->geom_quat + 4*g);
                cx += vr[0] + m->geom_pos[3*g+0];
                cy += vr[1] + m->geom_pos[3*g+1];
            }
            cx /= n; cy /= n;
            double band = 0.0015;  // 1.5 mm bands
            double rbot_max = 0, rtop_max = 0;
            for (int k = 0; k < n; ++k) {
                mjtNum v[3] = {m->mesh_vert[3*(adr+k)+0], m->mesh_vert[3*(adr+k)+1], m->mesh_vert[3*(adr+k)+2]};
                mjtNum vr[3]; mju_rotVecQuat(vr, v, m->geom_quat + 4*g);
                double xb = vr[0] + m->geom_pos[3*g+0], yb = vr[1] + m->geom_pos[3*g+1], zb = vr[2] + m->geom_pos[3*g+2];
                double r = std::sqrt((xb-cx)*(xb-cx) + (yb-cy)*(yb-cy));
                if (zb <= zmin + band && r > rbot_max) rbot_max = r;
                if (zb >= zmax - band && r > rtop_max) rtop_max = r;
            }
            printf("    -> hub check: max XY-radius near bottom=%.4f m, near top=%.4f m  (small=hub side)\n",
                   rbot_max, rtop_max);
            printf("    -> centroid xy=(%.4f, %.4f), z-center=%.4f\n",
                   cx, cy, 0.5 * (zmin + zmax));
        }
    }
    mj_deleteModel(m);
    return 0;
}
