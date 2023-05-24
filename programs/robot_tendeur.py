from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
import socket


def draw_robot(ax, p, R, alpha):
    a0, a1, a2 = alpha.flatten()
    T0 = tran3H(*p.flatten()) @ ToH(R)  # rotate and translate
    P0 = hstack((circle3H(blds), [[+blds, -blds], [0, 0], [0, 0], [1, 1]]))  # disc + blades

    C0 = tran3H(L / 2, 0, l) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a0) @ P0  # we rotate the blades
    C1 = tran3H(L / 2, l * sin((2 / 3) * pi), l * cos((2 / 3) * pi)) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a1) @ P0
    C2 = tran3H(L / 2, l * sin((4 / 3) * pi), l * cos((4 / 3) * pi)) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a2) @ P0

    draw3H(ax, T0 @ C0, 'green')
    draw3H(ax, T0 @ C1, 'red')
    draw3H(ax, T0 @ C2, 'red')

    M = T0 @ tran3H(-L / 2, 0, 0) @ cylinder3H(radius, L)
    ax.plot(M[0], M[1], 1 * M[2], color='orange')
    ax.plot(M[0], M[1], 0 * M[2], color='grey')  # draw shadow at sea level


def lest_torque(R):
    phi, theta, psi = eulermat2angles(R)
    return (-radius * m_lest * g) * array([[sin(phi)], [sin(theta)], [0]])


def update_robot(p, R, vr, wr, u):
    Fx, Cx, Cy, Cz = (B @ u).flatten()
    Fr = array([[Fx], [0], [0]]) - trans_fr * vr
    Cr = array([[Cx], [Cy], [Cz]]) + lest_torque(R) - rot_fr * wr

    p = p + dt * R @ vr
    R = R @ expw(dt * wr)

    vr = vr + dt * (Fr / m - adjoint(wr) @ vr)
    wr = wr + dt * inv(I) @ (Cr - adjoint(wr) @ (I @ wr))

    return p, R, vr, wr


def orientation_control(R, wr, R_d):
    w_rdp = root ** 2 * R.T @ logw(R_d @ R.T) - 2 * root * wr
    Cr = I @ w_rdp + adjoint(wr) @ I @ wr + lest_torque(R) - rot_fr * wr
    return inv(B[1:, :]) @ Cr


# def createSocket():
#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.bind(("172.20.20.88", 12345))
#     s.listen(5)
#     c, addr = s.accept()
#     print("Got connection from" + str(addr))
#     return c


if __name__ == '__main__':
    t, dt = 0, 0.05
    side = 1  # pool side in meters
    root = 3  # dynamic equation root

    p = array([[0], [0], [-side / 2]])  # 3D position : x,y,z (front,right,down)
    R = eulermat(0, 0, 0.1)  # rotation matrix
    vr = array([[0], [0], [0]])  # robot speed in its own frame
    wr = array([[0], [0], [0]])  # robot angular speed in its own frame

    alpha = array([[0], [0], [0]])  # angles for the blades
    R_d = eulermat(0, 0, pi/2)  # desired rotation matrix : cap command

    m = 4  # robot mass in kilograms
    g = 9.81  # gravitational constant in m.s-2
    m_lest = 0.5  # lest mass in kilograms

    trans_fr = 1e-2  # frictional resistance of water against translations
    rot_fr = 5e-2  # frictional resistance of water against rotations

    L = 0.3  # length of the robot in meters
    l = 0.2  # distance of propellers from robot axis
    radius = 0.05  # robot radius in meters
    blds = 0.1  # blades size

    # inertial matrix of the robot
    I = diag([(1 / 2) * m * radius ** 2,
              (1 / 4) * m * radius ** 2 + (1 / 12) * m * L ** 2,
              (1 / 4) * m * radius ** 2 + (1 / 12) * m * L ** 2])

    # Define links between propellers and mechanical actions
    kTx = 1e-2
    kCx = 1e-3
    kCy = 1e-1
    kCz = 1e-1

    # to convert commands to propellers speed
    B = array([[kTx, kTx, kTx],
               [-kCx * l, kCx * l, kCx * l],  # first motor turn in opposite direction
               [kCy * l, (-sqrt(3) / 2) * kCy * l, (-sqrt(3) / 2) * kCy * l],
               [0, (+1 / 2) * kCz * l, (-1 / 2) * kCz * l]])

    u = array([[0], [0], [0]])  # propeller angular speeds

    fig = figure()
    ax = fig.add_subplot(111, projection='3d')

    # c = createSocket()

    while True:

        t_loop = time.time()
        clean3D(ax, -side, +side, -side, +side, -side, 0)
        draw_robot(ax, p, R, alpha)

        u = orientation_control(R, wr, R_d)
        u1, u2, u3 = u[0, 0], u[1, 0], u[2, 0]
        # print(u1, u2, u3)
        # try:
        #     c.send((str(u1) + ' ' + str(u2) + ' ' + str(u3)).encode('utf-8'))
        # except ConnectionResetError:
        #     break

        p, R, vr, wr = update_robot(p, R, vr, wr, u)
        alpha = alpha + dt * u * 20

        t += dt
        delta_time = dt - (time.time() - t_loop)
        if delta_time > 0:
            pause(delta_time)
