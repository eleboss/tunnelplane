# #!/usr/bin/python2.7
# # coding=<encoding name> 例如，可添加# coding=utf-8
# def TakeOff(tko_x, tko_y, tko_z, current_z, dof2_x,dof2_y):
#     arm_mode = 1
#     if (current_z < tko_z):
#         if arm_status != 1:
#             arm_mode = 1
#         return [True, tko_x, tko_y, tko_z, arm_mode]
#     if (current_z > tko_z):
#         if arm_status == 1:
#             arm_mode = 2
#         return [False, tko_x, tko_y, tko_z, dof2_x,dof2_y, arm_mode]
# def Landing(lnd_x, lnd_y, lnd_z, current_z, dof2_x,dof2_y):
#     arm_mode = 0
#     if arm_status != 1:
#         arm_mode = 1
#         return [True, False, lnd_x, lnd_y, lnd_z, arm_mode]
#     if arm_status == 1:
#         arm_mode = 1
#         return [True, True, lnd_x, lnd_y, lnd_z, arm_mode]


