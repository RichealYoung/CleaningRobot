import helper
from gekko import GEKKO
import numpy as np
import time
import pandas as pd
from os.path import join as opj
from os.path import dirname as opd
import os
import time
def optimize_path(n_steps_for_rubbish,n_steps_for_obstacle,output_dir):
    # 1. Assigning task for every steps
    #init point
    x0=hW
    y0=hH
    step_task=[]
    clean_prior=helper.tsp_solve()
    #clean rubbishes task
    for i in range(len(clean_prior)):
        for j in range(n_steps_for_rubbish):
            step_task.append({'rubbish':clean_prior[i],'obstacle':None,'turn':False,})
    #avoid obstacles task
    for obstacle_avoid in helper.get_obstacle_avoid_list():
        for i in range(len(step_task)-1):
            if step_task[i]['rubbish']==obstacle_avoid['start_rubbish'] and step_task[i+1]['rubbish']==obstacle_avoid['end_rubbish']:
                for j in range(n_steps_for_obstacle):
                    step_task.insert(i+1,
                    {'rubbish':None,'obstacle':obstacle_avoid['obstacle'],'turn':False,
                    'init_value':obstacle_avoid['init_value'],
                    'lb':obstacle_avoid['lb'],
                    'ub':obstacle_avoid['ub']})
    n_steps=len(step_task)
    # 2. Setup Optimizer
    #init gekko
    m=GEKKO(remote=False)
    x=helper.Array(m.Var,(n_steps),'x',lb=0,ub=10)
    y=helper.Array(m.Var,(n_steps),'y',lb=0,ub=10)
    #set objective
    m.Obj(m.sqrt((x[0]-x0)**2+(y[0]-y0)**2))
    for i in range(n_steps-1):
        m.Obj(m.sqrt((x[i]-x[i+1])**2+(y[i]-y[i+1])**2))
    #set decision vars to clean rubbishes
    rubbishes_loc=helper.get_rubbishes()
    for i in range(n_steps):
        if step_task[i]['rubbish']!=None:
            rubbishe_loc=rubbishes_loc[step_task[i]['rubbish']]
            #set init value for decision vars to speed up
            x[i].value=rubbishe_loc[0]
            y[i].value=rubbishe_loc[1]
            #set lowerbounds and upperbouonds for decision vars to speed up
            x[i].lower=rubbishe_loc[0]-np.sqrt(dis_max_2)
            x[i].upper=rubbishe_loc[0]+np.sqrt(dis_max_2)
            y[i].lower=rubbishe_loc[1]-np.sqrt(dis_max_2)
            y[i].upper=rubbishe_loc[1]+np.sqrt(dis_max_2)
            #set constraints of cleaning rubbishes
            m.Equation((x[i]-rubbishe_loc[0])**2+(y[i]-rubbishe_loc[1])**2<=dis_max_2*0.999)# "*0.999" for improvement of accuracy
    #set decision vars to avoid obstacles
    obstacles_corners_loc=helper.get_obstacles()
    n_interp=20 #hyperparam
    for i in range(n_steps):
        if step_task[i]['obstacle']!=None:
            point_loc= obstacles_corners_loc[step_task[i]['obstacle'][0]][step_task[i]['obstacle'][1]]
            #set init value for decision vars to speed up
            x[i].value=point_loc[0]+step_task[i]['init_value'][0]
            y[i].value=point_loc[1]+step_task[i]['init_value'][1]
            #set lowerbounds and upperbouonds for decision vars to speed up
            x[i].upper=point_loc[0]+step_task[i]['ub'][0]
            x[i].lower=point_loc[0]+step_task[i]['lb'][0]
            y[i].upper=point_loc[1]+step_task[i]['ub'][1]
            y[i].lower=point_loc[1]+step_task[i]['lb'][1]
            #set constraints of avoiding obstacles
            for k in range(n_interp):
                k=k+1
                x_interp=x[i]+k/n_interp*(x[i+1]-x[i])
                y_interp=y[i]+k/n_interp*(y[i+1]-y[i])
                m.Equation((x_interp-point_loc[0])**2+(y_interp-point_loc[1])**2>=hW**2*1.0001)# "*1.0001" for improvement of accuracy
    #optional solver settings with APOPT
    m.solver_options = ['minlp_maximum_iterations 50000', \
                        # minlp iterations with integer solution
                        'minlp_max_iter_with_int_sol 50000', \
                        # treat minlp as nlp
                        'minlp_as_nlp 0', \
                        # nlp sub-problem max iterations
                        'nlp_maximum_iterations 500', \
                        # 1 = depth first, 2 = breadth first
                        'minlp_branch_method 1', \
                        # maximum deviation from whole number
                        'minlp_integer_tol 5e-2', \
                        # covergence tolerance
                        'minlp_gap_tol 1e-6',\
                        'constraint_convergence_tolerance   1e-6',\
                        'objective_convergence_tolerance 1e-6']
    m.options.SOLVER=1
    m.options.COLDSTART=2
    m.solve()
    m.options.COLDSTART=0
    m.solve(disp=False)
    #integrate decision vars to list for postprocessing
    x_list=[x[i].value[0] for i in range(len(x))]
    y_list=[y[i].value[0] for i in range(len(y))]
    x_list.insert(0,x0)
    y_list.insert(0,y0)
    #plot the path
    # path_distance=helper.cal_path_distance(x_list,y_list)
    # title='Best Path without turn angle constraint'
    # wrong_angle_list=helper.check_turn_angle(x_list,y_list,angle_max)
    # helper.plot(x_list,y_list,wrong_angle_list=wrong_angle_list,title=title,output_dir=output_dir)

    # 3. Adjust turn angle
    #repeat until there are no wrong turn angles
    for i in range(20):
        d=10**(-4-i*1)#hyperparam
        #check wrong turn angle
        wrong_angle_list=helper.check_turn_angle(x_list,y_list,angle_max)
        if len(wrong_angle_list)==0:
            break
        #adjust one wrong turn angle one time
        helper.adjust_turn_angle(x_list,y_list,wrong_angle_list[0],d)
    # 4.Final results
    path_distance=helper.cal_path_distance(x_list,y_list)
    theta_list=helper.generate_turn_angle(x_list,y_list)
    x_list,y_list,theta_list=helper.refine_angle(x_list,y_list,theta_list)
    title='Best Path Plan'
    helper.plot(x_list,y_list,theta=theta_list,title=title,output_dir=output_dir,plotrobot=True,)
    helper.save_path(x_list,y_list,theta_list,output_dir)
    print('We use {} steps with distacne:{}'.format(len(x_list),path_distance))
    # 5.Check all constraints
    # rubbishes_cleaned=helper.check_rubbish(x_list,y_list,dis_max_2=dis_max_2)
    # print('We cleaned rubbishes:',rubbishes_cleaned)
    # obstacles_avoid=helper.check_obstacle(x_list,y_list,hW**2)
    # print('We avoid obstacles',obstacles_avoid)
    # wrong_angle_list=helper.check_turn_angle(x_list,y_list,angle_max)
    # print('We take wrong turn angles:',wrong_angle_list)

if __name__=='__main__':
    # dir
    time_str=time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
    output_dir=opj(opd(__file__),'model_'+time_str)
    os.makedirs(output_dir)
    # init param
    global W,H,hW,hH,dis_max_2,angle_max
    W=0.57
    H=0.7
    hW=W/2
    hH=H/2
    dis_max_2=hW**2+hH**2
    angle_max=0.2*np.pi
    n_steps_for_rubbish=1
    n_steps_for_obstacle=8
    tic=time.time()
    optimize_path(n_steps_for_rubbish,n_steps_for_obstacle,output_dir)
    toc=time.time()
    print('Time consumes:{:.3f}s'.format(toc-tic))
