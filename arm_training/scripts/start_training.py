#!/usr/bin/env python

import rospy, rospkg, numpy, time
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from gym import wrappers


if __name__ == '__main__':

    rospy.init_node('armdrone_stabilize', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = StartOpenAI_ROS_Environment('ArmDroneStabilizeEnv-v0')
    rospy.logwarn("- Gym environment done -")
    print(env)
    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('arm_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    nepisodes = rospy.get_param("/armdrone/nepisodes")
    nsteps = rospy.get_param("/armdrone/nsteps")

    # Initialises the algorithm that we are going to use for learning
    # qlearn = qlearn.QLearn(actions=range(env.action_space.n),
    #                 alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    # initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    observation = env.reset()
    state = ''.join(map(str, observation))

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logdebug("############### START EPISODE=>" + str(x))
        
        cumulated_reward = 0  
        done = False
        # if qlearn.epsilon > 0.05:
        #     qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))
        
        # Show on screen the actual situation of the robot
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            rospy.loginfo("############### Start Step=>"+str(i))
            # Pick an action based on the current state
            # action = qlearn.chooseAction(state)
            # rospy.loginfo("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(0)
            rospy.loginfo(str(observation) + " " + str(reward))
            # cumulated_reward += reward
            # if highest_reward < cumulated_reward:
            #     highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
            # rospy.logwarn("############### state we were=>" + str(state))
            # rospy.logwarn("############### action that we took=>" + str(action))
            # rospy.logwarn("############### reward that action gave=>" + str(reward))
            # rospy.logwarn("############### State in which we will start nect step=>" + str(nextState))
            # qlearn.learn(state, action, reward, nextState)

            if not(done):
                state = nextState
            else:
                rospy.loginfo ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.loginfo("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            #rospy.sleep(2.0)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        # rospy.logfatal ( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    
    # rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
