import numpy as np
import time
from CFS_problem import *
import cvxopt
from cvxopt import matrix, solvers
from numpy import linalg as LA
import matplotlib.pyplot as plt
from cvxpy import *
import matplotlib.pyplot as plt
import imageio



# #Quadratic programming solver.
# def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
#     P = .5 * (P + P.T)  # make sure P is symmetric
#     args = [matrix(P), matrix(q)]
#     if G is not None:
#         args.extend([matrix(G), matrix(h)])
#         if A is not None:
#             args.extend([matrix(A), matrix(b)])
#     sol = cvxopt.solvers.qp(*args)
#     if 'optimal' not in sol['status']:
#         return None
#     return np.array(sol['x']).reshape((P.shape[1],))

COLOR = ['red', 'blue', 'darkorange', 'black', 'green', 'darkblue', 'aqua', 'purple', 'maroon']

def get_taylor_expansion_points(high_pri_pt, low_pri_pt, mini_distance):
    '''
    Args:
        high_pri_pt: Reference point 1 with higher priority.
        low_pri_pt: Reference point with lower priority.
        mini_distance: Safety margin between two cars.
    Return:
        new_ref: New reference points that can be used for taylor expansion.
    '''
    new_ref = np.zeros((4, 1))
    new_ref[0:2] = high_pri_pt
    dir = (low_pri_pt - high_pri_pt) / np.linalg.norm(low_pri_pt - high_pri_pt)
    new_ref[2:4] = new_ref[0:2] + dir * mini_distance
    return new_ref

def two_car_distance(path1, path2):
    '''
    Args:
        path1: Trajectory of car 1.
        path2: Trajectory of car 2.
    Return:
        distance: Distance between two cars at every time steps.
    '''
    nstep = path1.shape[0]
    distance = []
    for i in range(nstep):
        # print("----------", i, "----------")
        pts1 = path1[i, :]
        pts2 = path2[i, :]
        # print(pts1, pts2, pts1-pts2)
        dist = np.linalg.norm(pts1-pts2)
        # print(dist)
        distance.append(dist)
    print(np.min(distance))
    return distance

def get_velocity(path, dt):
    '''
    Args:
        path: Trajectory of a car.
    Return:
        velocity: Calculated velocity of the car based on the path.
    '''
    velocity = []
    nstep = path.shape[0]
    # print("Reference speed: {}".format(100/nstep))
    for i in range(nstep-1):
        vel_step = np.linalg.norm(path[i] - path[i+1]) / dt
        velocity.append(vel_step)
    return velocity

def get_line(x1, y1, x2, y2):
    '''
    Args:
        x1, y1: Point 1 on the line.
        x2, y2: Point 2 on the line.
    Return:
        coe: coe[0:2] is the k vector, coe[2] is the b, so that k[0]x+k[1]y+b=0.
    '''
    coe = np.zeros((3,1))
    coe[0] = y2 - y1
    coe[1] = -(x2 - x1)
    coe[2] = y2*(x2-x1) - x2*(y2-y1)
    return coe

def path_rendering(pathnew, num):
    '''
    Args:
        pathnew: Optimized result variable.
        num: Number of cars.
    Return:
        Render the GIF image of the cars' trajectory.
    '''
    nstep = int(pathnew.shape[0] / (num*2))
    car_path = np.zeros((num, nstep, 2))
    images = []
    for i in range(num):
        car_path[i][:, 0] = pathnew[2*i : : num*2]
        car_path[i][:, 1] = pathnew[2*i+1 : : num*2]
    
    fig = plt.figure()
    for i in range(nstep):
        x = np.zeros((num, 1))
        y = np.zeros((num, 1))
        plot_legend = []
        for j in range(num):
            x[j] = car_path[j][i, 0]
            y[j] = car_path[j][i, 1]
            plt.xlim(-10, 10)
            plt.ylim(-40, 130)
            plt.scatter(x[j], y[j], marker = 'o', color = COLOR[j])
            plot_legend.append('car {}'.format(j))
        plt.legend(plot_legend, loc = 'upper right')
        fig.savefig('./results/{}.png'.format(i))
        images.append(imageio.imread('./results/{}.png'.format(i)))
    imageio.mimsave('./results/Path_rendering.gif', images)


    
def Plan_trajectory(MAX_ITER, multi_path, mini_distance):
    '''
    Args:
        MAX_ITER: Maximum iterations of optimizations.
        multi_path: Shape: num_cars x nsteps x 2. Reference path of the multiple cars.
        mini_distance: Safety margin between two cars.
    Returns:
        pathnew: Planned path for multiple cars.
    '''
    Qref, Qabs, nstep, dim, oripath, I_2 = Setup_problem(multi_path)
    refpath = oripath
    print("refpath shape is:{}".format(refpath.shape))
    print("Qref shape:{}, Qabs shape:{}".format(Qref.shape, Qabs.shape))
    print("Dimension is: {}".format(dim))
    Qe = Qref + Qabs
    n = nstep * dim
    print("n is: {}".format(n))

    for i in range(MAX_ITER):
        print(i)
        x = Variable(n)
        objective = Minimize(0.5*quad_form(x,Qe) + (np.matmul(-Qref,oripath)).T @ x)
        constraints = []
        for j in range(nstep):
            x_ref_1 = refpath[dim*j:dim*(j+1)]
            # if j < nstep-1:
            #     x_ref_2 = refpath[dim*(j+1):dim*(j+2)]
            if j <= 1 or j >= nstep - 2:
            	constraints.append(x[dim*j:dim*(j+1)] - oripath.squeeze()[dim*j:dim*(j+1)] == 0)
            
            # Define distance constraint
            for l in range(int(dim/2)):                              # Loop through multiple cars
                for m in range(l+1, int(dim/2)):                     # Loop through other cars to define the distance constraint
                    ref_point_1 = x_ref_1[2*l : 2*(l+1)].reshape(2,1)
                    ref_point_2 = x_ref_1[2*m : 2*(m+1)].reshape(2,1)
                    ref_points = get_taylor_expansion_points(ref_point_1, ref_point_2, mini_distance)
                    A_step = 2 * ref_points.T @ I_2
                    b = -mini_distance**2 - 1/2 * (A_step @ ref_points)
                    A = -A_step

                    cons = A[0, 0:2].reshape(1,2)@x[dim*j:dim*(j+1)][2*l:2*(l+1)] + A[0, 2:4].reshape(1,2)@x[dim*j:dim*(j+1)][2*m:2*(m+1)] <= b
                    constraints.append(cons)
            
            # Define lane keeping constraint for the car being overtaked
            # lane_keeping_id = [0, 2]
            # for k in lane_keeping_id:
            #     cons = x[dim*j + 2*k+1] == x_ref_1[2*k+1]
            #     constraints.append(cons)
            
            
            # Define priority constraint
            '''
            For 9 car scenario, the reference trajectories don't have intersections at a specific same time, hence no need 
            for the priority constraints.
            '''
            # if j < nstep-1:
            #     coe1 = get_line(x_ref_1[0], x_ref_1[1], x_ref_2[0], x_ref_2[1])
            #     coe2 = get_line(x_ref_1[2], x_ref_1[3], x_ref_2[2], x_ref_2[3])
            #     k1 = coe1[0:2]
            #     b1 = coe1[2]
            #     k2 = coe2[0:2]
            #     b2 = coe2[2]
            #     #print("k shape: {}".format(k1.shape))
            #     if k1.T @ x_ref_1[2:4] + b1 > 0 and k1.T @ x_ref_2[2:4] + b1 < 0 and (k2.T @ x_ref_1[0:2] + b2)*(k2.T @ x_ref_2[0:2] + b2) < 0:
            #         cons1 = -k1.T @ x[dim*j+dim/2 : dim*(j+1)] <= b1
            #         cons2 = k1.T @ x[dim*(j+1)+dim/2 : dim*(j+2)] <= -b1
            #         constraints.append(cons1)
            #         constraints.append(cons2)
            #         if k2.T @ x_ref_1[0:2] + b2 > 0:
            #             cons3 = -k2.T @ x[dim*j : dim*j + dim/2] <= b2
            #             cons4 = -k2.T @ x[dim*(j+1) : dim*(j+1) + dim/2] <= b2
            #             constraints.append(cons3)
            #             constraints.append(cons4)
            #         else:
            #             cons3 = k2.T @ x[dim*j : dim*j + dim/2] <= -b2
            #             cons4 = k2.T @ x[dim*(j+1) : dim*(j+1) + dim/2] <= -b2
            #             constraints.append(cons3)
            #             constraints.append(cons4)
                
            #     elif k1.T @ x_ref_1[2:4] + b1 < 0 and k1.T @ x_ref_2[2:4] + b1 > 0 and (k2.T @ x_ref_1[0:2] + b2)*(k2.T @ x_ref_2[0:2] + b2) < 0:
            #         cons1 = k1.T @ x[dim*j+dim/2 : dim*(j+1)] <= -b1
            #         cons2 = -k1.T @ x[dim*(j+1)+dim/2 : dim*(j+2)] <= b1
            #         constraints.append(cons1)
            #         constraints.append(cons2)
            #         if k2.T @ x_ref_1[0:2] + b2 > 0:
            #             cons3 = -k2.T @ x[dim*j : dim*j + dim/2] <= b2
            #             cons4 = -k2.T @ x[dim*(j+1) : dim*(j+1) + dim/2] <= b2
            #             constraints.append(cons3)
            #             constraints.append(cons4)
            #         else:
            #             cons3 = k2.T @ x[dim*j : dim*j + dim/2] <= -b2
            #             cons4 = k2.T @ x[dim*(j+1) : dim*(j+1) + dim/2] <= -b2
            #             constraints.append(cons3)
            #             constraints.append(cons4)

        # constraints.append(-2 <= 0)
        p = Problem(objective, constraints)
        primal_result = p.solve(solver = CVXOPT)

        pathnew = x.value
        print("pathnew is of shape: {}".format(pathnew.shape))
        print("pathnew is {}".format(pathnew))

        diff = LA.norm(refpath - pathnew)
        print("diff is: ", diff)
        if diff < 0.001*nstep*dim:
            print("Converged at step {}".format(i))
            break

        refpath = pathnew

        # Show planned path
        # for i in range(2):
        #     x = pathnew[2*i : : 4]
        #     y = pathnew[2*i+1 : : 4]
        #     plt.plot(y, x)
        # plt.legend(('1', '2'))
        # plt.show()

    print("Loop finished!")

    return pathnew



def main():
    MAX_ITER = 10
    num_cars = 9
    dt = 0.2
    multi_path = define_path(dt)

    pathnew = Plan_trajectory(MAX_ITER, multi_path, 3)
    
    # nstep = int(pathnew.shape[0] / (num_cars*2))
    # pathnew1 = np.zeros((nstep, 2))
    # pathnew2 = np.zeros((nstep, 2))
    # pathnew1[:, 0] = pathnew[2 : : 6]
    # pathnew1[:, 1] = pathnew[3 : : 6]
    # pathnew2[:, 0] = pathnew[4 : : 6]
    # pathnew2[:, 1] = pathnew[5 : : 6]
    # print(pathnew1[0, :], pathnew2[0, :])
    # distance = two_car_distance(pathnew1, pathnew2)
    

    # Get original car 3's trajectory
    # ori_path3 = multi_path[-1]
    # distance = two_car_distance(pathnew1, ori_path3)
    # print(distance, np.argmin(distance))
    # print(ori_path3)

    path_rendering(pathnew, num_cars)
    # vel = get_velocity(pathnew2, dt)
    # print("Average speed is: {}".format(sum(vel)/len(vel)))
    # print(vel)
    

if __name__ == "__main__":
    main()
