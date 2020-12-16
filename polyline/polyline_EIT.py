import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import csv
import argparse


def main(args):
    print('args', args)
    dist_bw_placement = args.dist_bw_placement
    rebar_length = args.rebar_length

    print('dist_bw_placement= ', dist_bw_placement, '; rebar_length=', rebar_length)
    x_pts = []
    y_pts = []
    pts = []
    fig, ax = plt.subplots()
    plt.title('Click to create BSpline points')
    line, = plt.plot(x_pts, y_pts, marker="o")
    plt.ylim([-0.1, 0.7])
    plt.xlim([-0.6, 0.6])

    plt.gca().set_aspect(1.5, adjustable='box')
    total_linear_length = 0
    dist_from_last_pt = dist_bw_placement
    img = mpimg.imread('table.png')
    ax.imshow(img, extent=[-0.6, 0.6, -0.1, 0.7, ])

    def check_intersection(p1, p2, p3, p4):
        # print('p1.T + p2.T + p3.T + p4.T', p1.T + p2.T + p3.T + p4.T)

        mySet = set(tuple(i) for i in [p1, p2, p3, p4])
        # print('myset', mySet)
        if len(mySet) < 4:
            # One of the points intersects
            return True
        else:
            # Reference from https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect

            A1 = (p1[1] - p2[1]) / (p1[0] - p2[0])
            A2 = (p3[1] - p4[1]) / (p3[0] - p4[0])
            b1 = p1[1] - A1 * p1[0]  # = p2[1] - A1 * p2[0]
            b2 = p3[1] - A2 * p3[0]  # = p4[1] - A2 * p4[0]

            if A1 == A2:
                return False  # Parallel segments

            Xa = (b2 - b1) / (A1 - A2)
            print('A1', A1, 'b1', b1, 'Xa', Xa)
            Ya = A1 * Xa + b1  # = A2 * Xa + b2

            if ((Xa < max(min(p1[0], p2[0]), min(p3[0], p4[0]))) or (Xa > min(max(p1[0], p2[0]), max(p3[0], p4[0]))) or
                    Ya < max(min(p1[1], p2[1]), min(p3[1], p4[1])) or (Ya > min(max(p1[1], p2[1]), max(p3[1], p4[1])))):
                return False  # intersection is out of bounds
            else:
                return True

    def onpick(event):
        # global total_linear_length, dist_from_last_pt
        nonlocal dist_from_last_pt, total_linear_length
        m_x, m_y = event.x, event.y
        x, y = ax.transData.inverted().transform([m_x, m_y])
        if len(pts) > 0:
            dist_from_last_pt = np.sqrt((pts[-1][0] - x) ** 2 + (pts[-1][1] - y) ** 2)
        if dist_from_last_pt >= dist_bw_placement and -0.6 < x < 0.6 and -0.1 < y < 0.7:
            total_linear_length += dist_from_last_pt
            x_pts.append(x)
            y_pts.append(y)
            pts.append([x, y])
            line.set_xdata(x_pts)
            line.set_ydata(y_pts)
            fig.canvas.draw()
        else:
            print('Too close to previous point - dist_bw_placement=', dist_bw_placement)

    fig.canvas.mpl_connect('button_press_event', onpick)
    plt.show()
    pts = np.array(pts)
    number_of_intermediate_points = int(total_linear_length / dist_bw_placement)
    print('Total length = ', total_linear_length, '; number_of_intermediate_points', number_of_intermediate_points)

    def fabs(a):
        if a < 0:
            return -a
        return a

    x_der_y = []
    x_new = [];
    y_new = []
    xplot = [];
    yplot = []

    if len(x_pts) > 2:
        tck, u = splprep(pts.T, u=None, s=0.0, per=0, k=(2 if len(x_pts) == 3 else 3))  # B-spline
        # tck= (t,c,k) vector of knots, the B-spline coefficients, and the degree of the spline.
        # u = An array of the values of the parameter
        print('u', u)
        u_new = np.linspace(u.min(), u.max(), number_of_intermediate_points)
        xplot, yplot = splev(np.linspace(0, 1, 100), tck, der=0)
        x_new, y_new = splev(u_new, tck, der=0)
        x_der, y_der = splev(u_new, tck, der=1)

        x_der_y = np.arctan(y_der / x_der)
        print('x_der_y ', x_der_y * 180.0 / np.pi)
        print('x_der ', x_der)
        print('y_der ', y_der)

    else:
        slope = (y_pts[1] - y_pts[0]) / (x_pts[1] - x_pts[0])
        angle = np.arctan(slope)

        tot_segment_length = np.sqrt((y_pts[1] - y_pts[0]) ** 2 + (x_pts[1] - x_pts[0]) ** 2)
        number_of_intermediate_points = tot_segment_length / dist_bw_placement

        sign_x = 1 if x_pts[0] < x_pts[1] else -1
        sign_y = 1 if y_pts[0] < y_pts[1] else -1
        length_inc = 0
        for i in range(int(number_of_intermediate_points) + 1):
            x_new.append(x_pts[0] + sign_x * length_inc * np.cos(angle))
            y_new.append(y_pts[0] + sign_y * length_inc * np.sin(angle))
            length_inc += dist_bw_placement
            x_der_y.append(angle)
        plt.plot((x_pts[0], x_pts[1]), (y_pts[0], y_pts[1]), color='blue')
    plt.plot(x_pts, y_pts, 'bo')
    plt.plot(x_new, y_new, 'go', xplot, yplot)
    plt.title('Rebar placement')

    rebar_overlaps = []

    for i, pt in enumerate(x_der_y):
        x1 = x_new[i] - (rebar_length / 2.0) * np.cos(x_der_y[i] + np.pi / 2)
        x2 = x_new[i] + (rebar_length / 2.0) * np.cos(x_der_y[i] + np.pi / 2)

        y1 = y_new[i] - (rebar_length / 2.0) * np.sin(x_der_y[i] + np.pi / 2)
        y2 = y_new[i] + (rebar_length / 2.0) * np.sin(x_der_y[i] + np.pi / 2)

        if i > 0 and check_intersection(np.array([x1, y1]), np.array([x2, y2]), np.array([prev_x1, prev_y1]),
                                        np.array([prev_x2, prev_y2])):
            plt.plot((x1, x2), (y1, y2), color='red')
            rebar_overlaps.append(True)
        else:
            plt.plot((x1, x2), (y1, y2), color='green')
            rebar_overlaps.append(False)

        prev_x1 = x1;
        prev_x2 = x2
        prev_y1 = y1;
        prev_y2 = y2

    img = mpimg.imread('table.png')
    plt.imshow(img, extent=[-0.6, 0.6, -0.1, 0.7])

    with open('poses.csv', 'w', newline='') as csvfile:
        fieldnames = ['x', 'y', 'z_rot']
        writer = csv.writer(csvfile, delimiter=',')  # , fieldnames=fieldnames)
        #writer.writeheader()
        for i, pt in enumerate(x_der_y):
            if not rebar_overlaps[i]:
                # writer.writerow({'x': x_new[i], 'y': y_new[i], 'z_rot': x_der_y[i]})
                writer.writerow([x_new[i], y_new[i], x_der_y[i]])
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Creates a list of points on B-Spline and stores them in a csv file')
    parser.add_argument('dist_bw_placement', type=float, default=0.1,
                        help='distance between consecutive rebars (meters)')
    parser.add_argument('rebar_length', type=float, default=0.15,
                        help='distance between consecutive rebars (meters)')
    args = parser.parse_args()

    main(args)
