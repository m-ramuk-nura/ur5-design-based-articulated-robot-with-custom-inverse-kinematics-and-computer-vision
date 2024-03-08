import cv2
import numpy as np
from numpy import linalg
from math import pi
from math import cos
from math import sin
from math import atan2
from math import acos
from math import sqrt
from math import asin
import cv2
import numpy as np
from pyzbar.pyzbar import decode

DH_matrix = np.matrix([[0, pi / 2.0, 0.3],
                           [-0.415, 0, 0],
                           [-0.39243, 0, 0],
                           [0, pi / 2.0, 0.085],
                           [0, -pi / 2.0, 0.085],
                           [0, 0, 0.11]])


def translation_matrix(tx, ty, tz):
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])


def mat_transtorm_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    n = n - 1
    t_z_theta = np.matrix([[cos(edges[n]), -sin(edges[n]), 0, 0],
                           [sin(edges[n]), cos(edges[n]), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], copy=False)
    t_zd = np.matrix(np.identity(4), copy=False)
    t_zd[2, 3] = DH_matrix[n, 2]
    t_xa = np.matrix(np.identity(4), copy=False)
    t_xa[0, 3] = DH_matrix[n, 0]
    t_x_alpha = np.matrix([[1, 0, 0, 0],
                           [0, cos(DH_matrix[n, 1]), -sin(DH_matrix[n, 1]), 0],
                           [0, sin(DH_matrix[n, 1]), cos(DH_matrix[n, 1]), 0],
                           [0, 0, 0, 1]], copy=False)
    transform = t_z_theta * t_zd * t_xa * t_x_alpha
    return transform


def forward_kinematic_solution(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    t01 = mat_transtorm_DH(DH_matrix, 1, edges)
    t12 = mat_transtorm_DH(DH_matrix, 2, edges)
    t23 = mat_transtorm_DH(DH_matrix, 3, edges)
    t34 = mat_transtorm_DH(DH_matrix, 4, edges)
    t45 = mat_transtorm_DH(DH_matrix, 5, edges)
    t56 = mat_transtorm_DH(DH_matrix, 6, edges)
    answer = t01 * t12 * t23 * t34 * t45 * t56
    return answer


def inverse_kinematic_solution(DH_matrix, transform_matrix, ):
    theta = np.matrix(np.zeros((6, 8)))
    T06 = transform_matrix

    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
    psi = atan2(P05[1], P05[0])

    value = (P05[0] ** 2 + P05[1] ** 2 - (DH_matrix[1, 0] + DH_matrix[2, 0]) ** 2) / (
            2 * DH_matrix[1, 0] * DH_matrix[2, 0])
    valuee = (DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2)
    if -1 <= value <= 1:
        phi = np.arccos(valuee)
    else:
        phi = 0

    theta[0, 0:4] = psi + phi + pi / 2
    theta[0, 4:8] = psi - phi + pi / 2

    # theta 5
    for i in {0, 4}:
        th5cos = (T06[0, 3] * sin(theta[0, i]) - T06[1, 3] * cos(theta[0, i]) - (
                DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2])) / DH_matrix[5, 2]
        if 1 >= th5cos >= -1:
            th5 = acos(th5cos)
        else:
            th5 = 0
        theta[4, i:i + 2] = th5
        theta[4, i + 2:i + 4] = -th5

    # theta 6
    for i in {0, 2, 4, 6}:
        if sin(theta[4, i]) == 0:
            theta[5, i:i + 1] = 0
            break
        T60 = linalg.inv(T06)
        th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i])),
                   (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i])))
        theta[5, i:i + 2] = th

    # theta 3
    for i in [0, 2, 4, 6]:
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        costh3 = ((P13[0] ** 2 + P13[1] ** 2 - DH_matrix[1, 0] ** 2 - DH_matrix[2, 0] ** 2) /
                  (2 * DH_matrix[1, 0] * DH_matrix[2, 0]))

        if 1 >= costh3 >= -1:
            th3 = acos(costh3)
        else:
            th3 = 0
        theta[2, i] = th3
        theta[2, i + 1] = -th3

    # theta 2, 4
    for i in range(8):
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2))
        T32 = linalg.inv(mat_transtorm_DH(DH_matrix, 3, theta[:, i]))
        T21 = linalg.inv(mat_transtorm_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])

    return theta


def inverse_kinematic_solution2(DH_matrix, transform_matrix, ):
    theta = np.matrix(np.zeros((6, 8)))
    # theta 1
    T06 = transform_matrix

    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
    psi = atan2(P05[1], P05[0])
    phi = acos((DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2))
    theta[0, 0:4] = psi + phi + pi / 2
    theta[0, 4:8] = psi - phi + pi / 2

    # theta 5
    for i in [0, 4]:
        theta[4, i:i + 2] = atan2(sqrt(1 - T06[2, 2] ** 2), T06[2, 2])

    # theta 6
    for i in [0, 2, 4, 6]:
        T60 = linalg.inv(T06)
        theta[5, i:i + 2] = atan2(T60[2, 1], -T60[2, 0])

    # theta 3
    for i in [0, 2, 4, 6]:
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        costh3 = ((P13[0] ** 2 + P13[1] ** 2 - DH_matrix[1, 0] ** 2 - DH_matrix[2, 0] ** 2) /
                  (2 * DH_matrix[1, 0] * DH_matrix[2, 0]))

        if 1 >= costh3 >= -1:
            th3 = acos(costh3)
        else:
            th3 = 0
        theta[2, i] = th3
        theta[2, i + 1] = -th3

    # theta 2, 4
    for i in range(8):
        T01 = mat_transtorm_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transtorm_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transtorm_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2))
        T32 = linalg.inv(mat_transtorm_DH(DH_matrix, 3, theta[:, i]))
        T21 = linalg.inv(mat_transtorm_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])

    return theta


def calculate_joint_angles(x, y, z, rx, ry, rz):
    transform_matrix = np.matrix([[cos(ry) * cos(rz), -cos(ry) * sin(rz), sin(ry), x],
                                  [cos(rx) * sin(rz) + sin(rx) * sin(ry) * cos(rz),
                                   cos(rx) * cos(rz) - sin(rx) * sin(ry) * sin(rz), -sin(rx) * cos(ry), y],
                                  [sin(rx) * sin(rz) - cos(rx) * sin(ry) * cos(rz),
                                   sin(rx) * cos(rz) + cos(rx) * sin(ry) * sin(rz), cos(rx) * cos(ry), z],
                                  [0, 0, 0, 1]])

    joint_angles = inverse_kinematic_solution(DH_matrix, transform_matrix)

    return joint_angles


def calculate_joint_angles2(x, y, z, rx, ry, rz):
    transform_matrix = np.matrix([[cos(ry) * cos(rz), -cos(ry) * sin(rz), sin(ry), x],
                                  [cos(rx) * sin(rz) + sin(rx) * sin(ry) * cos(rz),
                                   cos(rx) * cos(rz) - sin(rx) * sin(ry) * sin(rz), -sin(rx) * cos(ry), y],
                                  [sin(rx) * sin(rz) - cos(rx) * sin(ry) * cos(rz),
                                   sin(rx) * cos(rz) + cos(rx) * sin(ry) * sin(rz), cos(rx) * cos(ry), z],
                                  [0, 0, 0, 1]])

    joint_angles2 = inverse_kinematic_solution2(DH_matrix, transform_matrix)

    return joint_angles2


if __name__ == '__main__':
    pixels_per_meter_x = 900
    pixels_per_meter_y = 200

    focal_length = 400

    robot_matrix = np.identity(4)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    while True:
        success, img = cap.read()
        for barcode in decode(img):
            myData = barcode.data.decode('utf-8')

            width_pixels = barcode.rect[2]

            depth_meters = (1.0 / pixels_per_meter_x) * (1.0 / width_pixels) * focal_length

            center_x_cm = (barcode.rect[0] + barcode.rect[2] // 2 - img.shape[1] // 2) / pixels_per_meter_x * 100
            center_y_cm = (barcode.rect[1] + barcode.rect[
                3]) / pixels_per_meter_y * 200 / 15  # Remove the negative sign
            translation = translation_matrix(center_x_cm, center_y_cm, depth_meters)
            robot_matrix = np.dot(robot_matrix, translation)

            center_x = barcode.rect[0] + barcode.rect[2] // 2
            center_y = barcode.rect[1] + barcode.rect[3] // 2

            center_x_int = int(center_x_cm)
            center_y_int = int(center_y_cm)
            cv2.circle(img, (center_x_int, center_y_int), 10, (100, 0, 255), -1)
            cv2.circle(img, (center_x, center_y), 10, (255, 0, 255), -1)

            print("xcm", center_x_cm, "ycm", center_y_cm + 35, "zcm", depth_meters * 10000 - 21)
            cv2.putText(img,
                        f"{myData} (x: {center_x_cm:.2f} cm, y: {center_y_cm:.2f} cm, depth: {depth_meters:.2f} m)",
                        (barcode.rect[0], barcode.rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)

        cv2.imshow('Result', img)
        cv2.waitKey(1)

    print("Robot's final position and orientation:")
    print(robot_matrix)

    x = -0.00
    y = 0.4693
    z = 0.3906

    rx = 0.0

    ry = 0.0
    rz = 0.0

    joint_angles = calculate_joint_angles(x, y, z, rx, ry, rz)
    joint_angles2 = calculate_joint_angles2(x, y, z, rx, ry, rz)
    joint_angles_degrees = joint_angles * (180.0 / pi)
    joint_angles_degrees2 = joint_angles2 * (180.0 / pi)

    joint1_degrees = joint_angles_degrees[0, 0]
    joint2_degrees = (joint_angles_degrees[1, 0]) * -1
    joint3_degrees = joint_angles_degrees[2, 0]
    joint4_degrees = joint_angles_degrees[3, 0]
    joint5_degrees = joint_angles_degrees[4, 0]
    joint6_degrees = joint_angles_degrees[5, 0]
    joint1_degrees2 = joint_angles_degrees2[0, 0]
    joint2_degrees2 = (joint_angles_degrees2[1, 0]) * -1
    joint3_degrees2 = joint_angles_degrees2[2, 0]
    joint4_degrees2 = joint_angles_degrees2[3, 0]
    joint5_degrees2 = joint_angles_degrees2[4, 0]
    joint6_degrees2 = joint_angles_degrees2[5, 0]

    print("Joint 1: {:.2f} degrees".format(joint1_degrees))
    print("Joint 2: {:.2f} degrees".format(joint2_degrees))
    print("Joint 3: {:.2f} degrees".format(joint3_degrees))
    print("Joint 4: {:.2f} degrees".format(joint4_degrees))
    print("Joint 5: {:.2f} degrees".format(joint5_degrees))
    print("Joint 6: {:.2f} degrees".format(joint6_degrees))

    print("Joint 1: {:.2f} degrees".format(joint1_degrees2))
    print("Joint 2: {:.2f} degrees".format(joint2_degrees2))
    print("Joint 3: {:.2f} degrees".format(joint3_degrees2))
    print("Joint 4: {:.2f} degrees".format(joint4_degrees2))
    print("Joint 5: {:.2f} degrees".format(joint5_degrees2))
    print("Joint 6: {:.2f} degrees".format(joint6_degrees2))

