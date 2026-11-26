"""Anzeige: Kartenfenster (Draufsicht) und 3D-Ansicht."""
import math

import cv2
import numpy as np

from config import *  # noqa: F401,F403

def draw(grid, pose, traj, scan_world, info):
    img = cv2.cvtColor(grid.image(), cv2.COLOR_GRAY2BGR)
    n = grid.n

    def px(xy):
        c = grid.to_cell(np.atleast_2d(xy))
        return np.column_stack((c[:, 0], n - 1 - c[:, 1]))

    if len(scan_world):
        q = px(scan_world)
        ok = (q[:, 0] >= 0) & (q[:, 0] < n) & (q[:, 1] >= 0) & (q[:, 1] < n)
        img[q[ok, 1], q[ok, 0]] = (0, 200, 0)
    if len(traj) > 1:
        cv2.polylines(img, [px(np.array(traj)[:, :2]).astype(np.int32)], False, (0, 0, 255), 1)
    p = px(pose[:2])[0]
    tip = px(np.array(pose[:2]) + 0.5 * np.array([math.cos(pose[2]), math.sin(pose[2])]))[0]
    cv2.arrowedLine(img, tuple(int(v) for v in p), tuple(int(v) for v in tip), (255, 0, 0), 2, tipLength=0.4)

    half = int(min(n, max(200, 2 * MAX_RANGE / grid.res)) / 2)
    cx = int(np.clip(p[0], half, n - half))
    cy = int(np.clip(p[1], half, n - half))
    view = img[cy - half:cy + half, cx - half:cx + half]
    view = cv2.resize(view, (WINDOW_PX, WINDOW_PX), interpolation=cv2.INTER_NEAREST)
    cv2.putText(view, info, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return view


def show_3d(points, traj_xy=None):
    """3D-Ansicht: open3d, falls installiert, sonst matplotlib."""
    if len(points) == 0:
        print("Noch keine 3D-Punkte.")
        return
    try:
        import open3d as o3d
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        z = points[:, 2]
        c = cv2.applyColorMap(((z - z.min()) / max(np.ptp(z), 1e-6) * 255).astype(np.uint8)[:, None],
                              cv2.COLORMAP_TURBO)[:, 0, ::-1] / 255.0
        pc.colors = o3d.utility.Vector3dVector(c)
        o3d.visualization.draw_geometries([pc], window_name="3D-Karte")
        return
    except ImportError:
        pass
    import matplotlib.pyplot as plt
    p = points if len(points) < 60000 else points[np.random.default_rng(0).choice(len(points), 60000, False)]
    fig = plt.figure("3D-Karte", figsize=(9, 7))
    ax = fig.add_subplot(projection="3d")
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=p[:, 2], cmap="turbo", s=1)
    if traj_xy is not None and len(traj_xy):
        ax.plot(traj_xy[:, 0], traj_xy[:, 1], np.zeros(len(traj_xy)), "k-", lw=0.8)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    # gleiche Achsenskalierung
    mid = (p.max(0) + p.min(0)) / 2
    r = (p.max(0) - p.min(0)).max() / 2
    ax.set_xlim(mid[0] - r, mid[0] + r); ax.set_ylim(mid[1] - r, mid[1] + r)
    ax.set_zlim(max(0, mid[2] - r), mid[2] + r)
    plt.show()
