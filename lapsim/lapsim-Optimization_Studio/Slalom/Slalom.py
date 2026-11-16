"""
透過連續過彎路徑特徵路段優化前輪輪距
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Arc


def slalom_path_analysis(
    track_width=10,
    offset_list=None,
    r_cone=0.16122034611053285,
    Driving_error=0.2,
    gap=7,
    L=1.53,
    best_wheel_track=1.280,
    num_circles=5,
    show_plot=True,
    save_path="slalom_path.png",
):
    """
    透過連續過彎路徑特徵分析前輪輪距

    Parameters
    ----------
    track_width : float
        賽道寬度

    offset_list : list
        offset比例

    r_cone : float
        角錐半徑

    Driving_error : float
        駕駛誤差

    gap : float
        角錐間距

    L : float
        軸距

    best_wheel_track : float
        前輪輪距

    num_circles : int
        顯示角錐數量

    show_plot : bool
        True:
            建立圖形、顯示並存檔
        False:
            完全跳過繪圖流程

    save_path : str
        圖片儲存位置

    Returns
    -------
    Curve_radius_list : ndarray
        各offset對應最佳過彎半徑
    """

    # ===============================================
    # 參數設定
    # ===============================================

    if offset_list is None:
        offset_list = [
            0.01,
            0.16666666666666666,
            0.3333333333333333
        ]

    offsets = track_width * np.array(offset_list)


    # 不包含輪距的半徑量
    d_lim = (
        190.5 / 2 / 1000
        + r_cone
        + Driving_error
    )

    best_r_lim = (
        best_wheel_track / 2
        + d_lim
    )


    # ===============================================
    # 建立畫布
    # (只有show_plot=True才建立)
    # ===============================================

    if show_plot:

        fig, axs = plt.subplots(
            len(offsets),
            1,
            figsize=(15, 5),
            constrained_layout=True
        )

        if len(offsets) == 1:
            axs = [axs]


    Curve_radius_list = []


    # ===============================================
    # 走線分析
    # ===============================================

    for ax_idx in range(len(offsets)):


        this_offset = offsets[ax_idx]


        # compute curve radius
        Curve_radius = (
            (gap / 2) ** 2
            + (this_offset + best_r_lim) ** 2
        ) / (
            2 * (this_offset + best_r_lim)
        )


        Curve_radius_list.append(Curve_radius)


        print(
            "最佳過彎半徑:",
            Curve_radius
        )


        offset_theta = np.rad2deg(
            np.arctan(
                (
                    2
                    *
                    (
                        Curve_radius
                        - this_offset
                        - best_r_lim
                    )
                )
                /
                gap
            )
        )


        # ===========================================
        # 繪圖
        # ===========================================

        if show_plot:

            ax = axs[ax_idx]


            for i in range(num_circles):


                off = this_offset * ((-1) ** i)


                x = i * gap
                y = off


                circle = Circle(
                    (x, y),
                    best_r_lim,
                    edgecolor='blue',
                    facecolor='none',
                    linewidth=2
                )

                ax.add_patch(circle)



                # determine arc angles depending on sign
                if off < 0:

                    theta1 = 180 + offset_theta
                    theta2 = 0 - offset_theta

                else:

                    theta1 = 0 + offset_theta
                    theta2 = 180 - offset_theta



                arc_center_y = (
                    off
                    +
                    (
                        best_r_lim
                        -
                        Curve_radius
                    )
                    *
                    np.sign(off)
                )


                arc = Arc(
                    (
                        x,
                        arc_center_y
                    ),
                    width=Curve_radius * 2,
                    height=Curve_radius * 2,
                    theta1=theta1,
                    theta2=theta2,
                    linewidth=2
                )


                ax.add_patch(arc)



            # subplot settings
            ax.set_xlabel('X')
            ax.set_ylabel('Y')

            ax.set_title(
                f'slalom '
                f'(offset index={offsets[ax_idx]})'
            )

            ax.grid(True)

            ax.autoscale()

            ax.set_xlim(
                -gap,
                gap*num_circles+1
            )

            ax.set_aspect('equal')



    # ===============================================
    # 儲存圖片
    # ===============================================

    if show_plot:

        plt.savefig(
            save_path,
            dpi=300,
            bbox_inches="tight"
        )

        plt.show()

        plt.close(fig)


    return np.array(Curve_radius_list)



# =====================================================
# Main
# =====================================================

if __name__ == "__main__":


    Curve_radius = slalom_path_analysis(show_plot=True)


    print("==============================")
    print("Curve Radius:")
    print(Curve_radius)