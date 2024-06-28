import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
import csv

import os
import math

agent_positions_x = {}
agent_positions_y = {}

def collect_agent_position(agent_id,position):
    if agent_id not in agent_positions_x:
        agent_positions_x[agent_id] = []
        agent_positions_y[agent_id] = []
    agent_positions_x[agent_id].append(position[0])
    agent_positions_y[agent_id].append(position[1])



def plot(leader_agents,follower_agents):
    global agent_positions_x,agent_positions_y

    leader_color='blue'
    follower_color='red'
    leader_list=[]
    follower_list=[]
    for l in leader_agents:
        leader_list.append(l.cf_id)
    for f in follower_agents:
        follower_list.append(f.cf_id)
    agents_list=leader_list+follower_list
    for a in agents_list:
          if a not in agent_positions_x:
              continue
          pos_x=agent_positions_x[a]
          pos_y=agent_positions_y[a]

          if(a in leader_list):
              plt.plot(pos_x,pos_y,leader_color)
          if(a in follower_list):
              plt.plot(pos_x,pos_y,follower_color)
    
    for l_i in range(len(leader_agents)):
        
        end=l_i+1
        end=end%len(leader_agents)
        plt.plot([leader_agents[l_i].position[0],leader_agents[end].position[0]],
                 [leader_agents[l_i].position[1],leader_agents[end].position[1]],leader_color)
    
    for f_i in follower_agents:
        plt.scatter([f_i.position[0]],[f_i.position[1]],s=20,color="green")
        

    plt.show()




def save_agent_positions_to_csv(filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header row
        writer.writerow(['Agent_ID', 'X_Positions', 'Y_Positions'])
        # Write data rows
        for agent_id, x_positions in agent_positions_x.items():
            y_positions = agent_positions_y[agent_id]
            writer.writerow([agent_id, x_positions, y_positions])
"""
def save_agent_positions_to_csv(filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header row
        writer.writerow(['Agent_ID', 'X_Positions', 'Y_Positions'])
        # Write data rows
        for agent_id, x_positions in agent_positions_x.items():
            y_positions = agent_positions_y[agent_id]
            # Write agent ID in the first column
            writer.writerow([agent_id])
            # Write X and Y positions alternatively in the second and third columns
            for x, y in zip(x_positions, y_positions):
                writer.writerow(['', x, y])



def save_agent_positions_to_csv(filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header row
        writer.writerow(['Agent_ID', 'X_Position', 'Y_Position'])
        # Write data rows
        for agent_id, x_positions in agent_positions_x.items():
            y_positions = agent_positions_y[agent_id]
            num_positions = min(len(x_positions), len(y_positions))
            # Write X and Y positions side by side
            for i in range(num_positions):
                writer.writerow([agent_id, x_positions[i], y_positions[i]])


import csv

def save_agent_positions_to_csv(filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write data rows
        for agent_id, x_positions in agent_positions_x.items():
            y_positions = agent_positions_y[agent_id]
            num_positions = min(len(x_positions), len(y_positions))
            # Duplicate the agent ID in alternating columns
            agent_ids = [agent_id] * num_positions * 2
            x_values = x_positions * 2
            y_values = y_positions * 2
            # Interleave the X and Y positions
            data_rows = [val for pair in zip(x_values, y_values) for val in pair]
            # Write the data row
            writer.writerow(agent_ids + data_rows)



# Call the function to save agent positions to a CSV file



# Initialize the figure
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)  # Adjust the limits as needed
ax.set_ylim(-10, 10)

# Initialize empty plots for agents
agent_plots = {}

def init():
    # Create plots for each agent
    for agent_id in agent_positions_x.keys():
        agent_plots[agent_id], = ax.plot([], [], marker='o', label=f'Agent {agent_id}')
    return list(agent_plots.values())

def update(frame):
    # Update positions for each agent
    for agent_id, plot in agent_plots.items():
        plot.set_data(agent_positions_x[agent_id][:frame], agent_positions_y[agent_id][:frame])
    return list(agent_plots.values())

def get_animate_object():
    # Create the animation
    ani = FuncAnimation(fig, update, frames=8000,
                        init_func=init, blit=True, repeat=False)
    # Add legend
    ax.legend()

    return ani

def animate(ani):
    ani=ani
    plt.show()
    current_directory = os.getcwd()
    save_path = os.path.join(current_directory, r"simple_animation.gif")

    # Save the animation as an animated GIF
    #ani.save(save_path, dpi=300,
        # writer=PillowWriter(fps=1))
   
    




"""



