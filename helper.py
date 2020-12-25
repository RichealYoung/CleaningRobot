from tkinter import Label
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from matplotlib.ticker import AutoMinorLocator, MultipleLocator
import math
from os.path import join as opj
import json
W=0.57
H=0.7
hW=W/2
hH=H/2
r=1+0.1 # scale #r range between lb and ub
def get_rubbishes_()->np.ndarray:
    '''
    Return the location(x,y) of all rubbish_points with shape=[15,2]
    '''
    rubbishes_loc=np.array(
    [
        [0, 0],
        [0.4,3],
        [2.9,2.2],
        [1.1,6.5],
        [2.9,5],
        [3,8.3],
        [5.2,4.8],
        [7.4,7.4],
        [5.3,1.2],
        [7.8,2.6],
        [6,6],
        [9,4.8],
        [5,8.5],
        [7,1.5],
        [2.5,7.5],
    ]
    )
    return rubbishes_loc

def get_rubbishes()->np.ndarray:
    '''
    Return the location(x,y) of all rubbish_points with shape=[14,2]
    '''
    rubbishes_loc=np.array(
    [
        [0.4,3],
        [2.9,2.2],
        [1.1,6.5],
        [2.9,5],
        [3,8.3],
        [5.2,4.8],
        [7.4,7.4],
        [5.3,1.2],
        [7.8,2.6],
        [6,6],
        [9,4.8],
        [5,8.5],
        [7,1.5],
        [2.5,7.5],
    ]
    )
    return rubbishes_loc

def get_obstacles()->list:
    '''
    Return the list of location(x,y) of corners for obstacles with shape=[4]
    '''
    obstacles_corners_loc= [
        np.array([[1.23,3.47],[1.4,2.67],[1.58,2.3],[2.1,3.63],[1.75,4]]),
        np.array([[4,6.48],[4.52,7.68],[5.06,7.73],[5.9,6.95],[4.65,5.98]]),
        np.array([[6.78,3.4],[7.78,3.76],[7.78,5.1]]),
        np.array([[4,3],[4.35,3.35],[4.8,3.45],[4.37,2.75]]),
    ]
    return obstacles_corners_loc
def get_obstacle_avoid_list()->list:
    obstacle_avoid_list=[
    {'start_rubbish':0,'end_rubbish':1,'obstacle':[0,2],'init_value':[0,-hW],'lb':[-hW*r,-hW*r],'ub':[hW*r,0]},
    {'start_rubbish':8,'end_rubbish':10,'obstacle':[2,1],'init_value':[hW,0],'lb':[-hW*r,-hW*r],'ub':[hW*r,hW*r]}
    ]
    return obstacle_avoid_list
def centers2corners(x:np.ndarray,y:np.ndarray,theta:np.ndarray)->np.ndarray:
    '''
    Return all the location(x,y) of four corners with shape=[:,4,2]
    '''
    assert len(x)==len(y) and len(y)==len(theta)
    corners=np.zeros((len(x),4,2))
    k=np.tan(theta)
    Deltax=np.sqrt(hH**2/(1+k**2))
    Deltay=k*Deltax
    dy=np.sqrt(hW**2/(1+k**2))
    dx=k*dy
    corners[:,0,0]=x+Deltax-dx
    corners[:,0,1]=y+Deltay+dy
    corners[:,1,0]=x+Deltax+dx
    corners[:,1,1]=y+Deltay-dy
    corners[:,2,0]=x-Deltax+dx
    corners[:,2,1]=y-Deltay-dy
    corners[:,3,0]=x-Deltax-dx
    corners[:,3,1]=y-Deltay+dy
    return corners
################
# matplotlib
################
def plot(x,y,plotpath=True,plotrobot=False,theta=None,title=' ',wrong_angle_list=[],output_dir=None,
                            xmin=0,xmax=0,ymin=0,ymax=0,)->None:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_aspect('equal')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))
    ax.xaxis.set_minor_locator(AutoMinorLocator(5))
    ax.yaxis.set_minor_locator(AutoMinorLocator(5))
    ax.grid(which='major', color='#CCCCCC', linestyle='--',alpha=0.8)
    ax.grid(which='minor', color='#CCCCCC', linestyle=':',alpha=0.4)
    ax.set_axisbelow(True)
    # plot rubbishes
    rubbishes_loc=get_rubbishes()
    ax.plot(rubbishes_loc[:,0],rubbishes_loc[:,1],'go',markersize=2)
    # plot obstacles
    patches=[]
    obstacles_corners_loc=get_obstacles()
    for obstacle_corners_loc in obstacles_corners_loc:
        polygon = Polygon(obstacle_corners_loc,False)
        patches.append(polygon)
    p = PatchCollection(patches, alpha=1)
    ax.add_collection(p)
    # plot robot path
    if plotpath:
        l1,=plt.plot(x,y,'--r',linewidth=0.5)
    # plot robot area
    if plotrobot:
        if theta==None:
            theta=[0 for i in range(x)]
        patches=[]
        all_robot_corners_loc=centers2corners(x,y,theta)
        for robot_corners_loc in all_robot_corners_loc:
            polygon = Polygon(robot_corners_loc,True,edgecolor='k',linewidth=0.15,facecolor='red')
            l3=ax.add_patch(polygon)
    # plot wrong_angle_list
    if len(wrong_angle_list):
        for wrong_angle in wrong_angle_list:
            l2,=plt.plot(x[wrong_angle+1],y[wrong_angle+1],'xk')
        if plotrobot:
            plt.legend(handles=[l1,l2,l3],
                    labels=['Best Path','Unsatisfied turn angle','Robot'])
        else:
            plt.legend(handles=[l1,l2],labels=['Best Path','Unsatisfied turn angle'])
    else:
        if plotrobot:
            plt.legend(handles=[l1,l3],
                    labels=['Best Path','Robot'])
        else:
            plt.legend(handles=[l1],labels=['Best Path'])
    plt.title(title)
    # xylim
    if xmin and xmax and ymin and ymax:
        plt.xlim(xmin,xmax)
        plt.ylim(ymin,ymax)
    # save figure
    if output_dir!=None:
        plt.savefig(opj(output_dir,title+'.png'),dpi=500)
        print('Best Path Plan.png is saved in {}'.format(output_dir))
    plt.close()
###############
#     tsp
##############
def cal_Distance(rubbishes_loc):
    L = rubbishes_loc.shape
    temp_1 = rubbishes_loc.reshape([L[0], 1, 2])
    temp_1 = np.repeat(temp_1, L[0], axis=1)
    temp_2 = rubbishes_loc.reshape([1, L[0], 2])
    temp_2 = np.repeat(temp_2, L[0], axis=0)
    dxdy = temp_1 - temp_2
    DistanceMatirx = dxdy[:, :, 0]*(dxdy[:, :, 0]) + dxdy[:, :, 1]*(dxdy[:, :, 1])
    DistanceMatirx = np.sqrt(DistanceMatirx.flatten()).reshape([L[0], L[0]])
    return DistanceMatirx
def cal_Next(Flag_collect, CurrentRoute, CurrentDistance, DistanceMatirx, AllRoute, AllDistance):
    # collect
    LastNum = np.sum(Flag_collect)
    Currentrubbish = CurrentRoute[LastNum]
    Flag_collect[Currentrubbish] = 1
    CurrentDistance = CurrentDistance + DistanceMatirx[CurrentRoute[LastNum-1], Currentrubbish]
    CurrentNum = LastNum + 1
    # finish collect？
    if CurrentNum == Flag_collect.shape[0]:
        AllRoute = np.r_[AllRoute, [CurrentRoute]]
        AllDistance = np.r_[AllDistance, [CurrentDistance]]
        return AllRoute, AllDistance
    # Next
    DistanceRow = DistanceMatirx[:, Currentrubbish] + Flag_collect * 100
    order = np.argsort(DistanceRow).astype('int8')
    BestN = 2
    if CurrentNum > 10:
        BestN = 2
    BestN = int(np.minimum(BestN, Flag_collect.shape[0] - CurrentNum - 1))
    BestN = int(np.maximum(BestN, 1))
    for i_ in range(0, BestN):
        i = int(order[i_])
        if DistanceRow[i] > 1.5*DistanceRow[order[0]]:
            continue
        CurrentRoute[CurrentNum] = i
        AllRoute, AllDistance = cal_Next(Flag_collect * 1, CurrentRoute * 1, CurrentDistance, DistanceMatirx, AllRoute, AllDistance)
    return AllRoute, AllDistance
def tsp_solve():
    rubbishes_loc = get_rubbishes_()
    L = rubbishes_loc.shape
    DistanceMatirx  = cal_Distance(rubbishes_loc)
    orderMatirx = np.argsort(DistanceMatirx).astype('int8')
    Flag_collect = np.zeros(L[0]).astype('int8')
    CurrentRoute = np.zeros(L[0]).astype('int8')-1
    CurrentDistance = 0
    CurrentRoute[0] = 0
    AllRoute = np.zeros([1, L[0]]).astype('int8')-1
    AllDistance = np.zeros([1])-1
    AllRoute, AllDistance = cal_Next(Flag_collect, CurrentRoute, CurrentDistance, DistanceMatirx, AllRoute, AllDistance)
    orderAllDistance = np.sort(AllDistance)
    order = np.argsort(AllDistance)
    orderAllRoute = AllRoute[order, :]
    for i in range(1, 2):
        TheRoute = AllRoute[order[i], :]
        x = rubbishes_loc[TheRoute, 0]
        y = rubbishes_loc[TheRoute, 1]
        theta = y*0
    return orderAllRoute[1,1:]-1
################
#    gekko
################
def Array(f,dim,name,**args):
    x = np.ndarray(dim,dtype=object)
    for ind,i in enumerate(np.nditer(x, flags=["refs_ok"],op_flags=['readwrite'])):
        i[...] = f(name=name+'_{}'.format(ind),**args)
    return x
def Matrix(f,dim,name,**args):
    assert(len(dim))==2
    x = np.ndarray(dim,dtype=object)
    for ind,i in enumerate(np.nditer(x, flags=["refs_ok"],op_flags=['readwrite'])):
        dim_0=ind//dim[1]
        dim_1=ind%dim[1]
        i[...] = f(name=name+'_{}_{}'.format(dim_0,dim_1),**args)
    return x
##################
#     angle
#################
def generate_turn_angle(x_list,y_list):
    k_list=[(y_list[i+1]-y_list[i])/(x_list[i+1]-x_list[i]) for i in range(len(x_list)-1)]
    k_list.append(k_list[-1])
    theta_list=[math.atan(k) for k in k_list]
    theta_list[0]=math.pi/2
    return theta_list
def adjust_turn_angle(x_list,y_list,wrong_angle,d):
    #point1->point2->point3
    #point1->pointa->pointb->point3
    len21=np.sqrt((x_list[wrong_angle+1]-x_list[wrong_angle])**2+(y_list[wrong_angle+1]-y_list[wrong_angle])**2)
    len23=np.sqrt((x_list[wrong_angle+2]-x_list[wrong_angle+1])**2+(y_list[wrong_angle+2]-y_list[wrong_angle+1])**2)
    one_21_x=(x_list[wrong_angle]-x_list[wrong_angle+1])/len21
    one_21_y=(y_list[wrong_angle]-y_list[wrong_angle+1])/len21
    one_23_x=(x_list[wrong_angle+2]-x_list[wrong_angle+1])/len23
    one_23_y=(y_list[wrong_angle+2]-y_list[wrong_angle+1])/len23
    pointa_x=x_list[wrong_angle+1]+d*one_21_x
    pointa_y=y_list[wrong_angle+1]+d*one_21_y
    pointb_x=x_list[wrong_angle+1]+d*one_23_x
    pointb_y=y_list[wrong_angle+1]+d*one_23_y
    x_list.pop(wrong_angle+1)
    y_list.pop(wrong_angle+1)
    x_list.insert(wrong_angle+1,pointb_x)
    y_list.insert(wrong_angle+1,pointb_y)
    x_list.insert(wrong_angle+1,pointa_x)
    y_list.insert(wrong_angle+1,pointa_y)
def cal_path_distance(x_list,y_list):
    dis=[]
    for i in range(len(x_list)-1):
        dis.append(np.sqrt((x_list[i]-x_list[i+1])**2+(y_list[i]-y_list[i+1])**2))
    total_dis=sum(dis)
    return total_dis
def refine_angle(x_list,y_list,theta_list):
    rubbishes_loc=get_rubbishes()
    rubbishes_cleaned=-np.ones(len(rubbishes_loc))
    insert_value_list=[]
    for ind in range(len(x_list)):
        x=x_list[ind]
        y=y_list[ind]
        for i in range(len(rubbishes_loc)):
            if rubbishes_cleaned[i]==-1:
                if (x-rubbishes_loc[i][0])**2+(y-rubbishes_loc[i][1])**2<=(hW**2+hH**2):
                    rubbishes_cleaned[i]=ind+1
                    insert_value_list.append([x,y,math.atan((rubbishes_loc[i][1]-y)/(rubbishes_loc[i][0]-x))+math.atan(hW/hH)])
    for i,ind in enumerate(np.sort(rubbishes_cleaned)):
        ind=int(ind)+i
        x_list.insert(ind,insert_value_list[i][0])
        y_list.insert(ind,insert_value_list[i][1])
        theta_list.insert(ind,insert_value_list[i][2])
    return x_list,y_list,theta_list
###################
#    check
##################
def check_rubbish(x_list,y_list,dis_max_2):
    rubbishes_loc=get_rubbishes()
    rubbishes_cleaned=np.zeros(len(rubbishes_loc))
    for x,y in zip(x_list,y_list):
        for i in range(len(rubbishes_loc)):
            if (x-rubbishes_loc[i][0])**2+(y-rubbishes_loc[i][1])**2<=dis_max_2:
                rubbishes_cleaned[i]=1
    return rubbishes_cleaned
    
def check_obstacle(x_list,y_list,dis_min_2):
    n_interp=1000
    obstacles_corners_loc=get_obstacles()
    obstacles_avoid=np.ones(len(obstacles_corners_loc))
    for ind in range(len(x_list)-1):
        for i in range(len(obstacles_corners_loc)):
            for j in range(len(obstacles_corners_loc[i])):
                point_x=obstacles_corners_loc[i][j][0]
                point_y=obstacles_corners_loc[i][j][1]
                for k in range(n_interp):
                    x_interp=x_list[ind]+k/n_interp*(x_list[ind+1]-x_list[ind])
                    y_interp=y_list[ind]+k/n_interp*(y_list[ind+1]-y_list[ind])
                    if (x_interp-point_x)**2+(y_interp-point_y)**2<dis_min_2:
                        obstacles_avoid[i]=0
    return obstacles_avoid
def check_turn_angle(x_list,y_list,angle_max):
    #point1->point2->point3, calculate angle2
    wrong_angle_list=[]
    for i in range(len(x_list)-2):
        point1_x=x_list[i]
        point1_y=y_list[i]
        point2_x=x_list[i+1]
        point2_y=y_list[i+1]
        point3_x=x_list[i+2]
        point3_y=y_list[i+2]
        len21=np.sqrt((point2_x-point1_x)**2+(point2_y-point1_y)**2)
        len23=np.sqrt((point2_x-point3_x)**2+(point2_y-point3_y)**2)
        len31=np.sqrt((point3_x-point1_x)**2+(point3_y-point1_y)**2)
        cos_angle2=max(-1,(len21**2+len23**2-len31**2)/(2*len21*len23))
        angle2=math.acos(cos_angle2)
        if angle2<np.pi-angle_max:
            wrong_angle_list.append(i)
    return wrong_angle_list
###################
#    io
##################
def save_path(x_list,y_list,theta_list,output_dir):
    path={'x':x_list,'y':y_list,'theta':theta_list}
    with open(opj(output_dir,'Best Path Plan.json'),'w') as f:
        json.dump(path,f,indent=2)
    print('Best Path Plan.json is saved in {}'.format(output_dir))
