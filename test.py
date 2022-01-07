import matplotlib.pyplot as plt

def show_plot(result):
    plt.figure(figsize=(15, 4))
    plt.plot(result.keys(), result.values(),
             )
    # label='%s planning steps' % planning_step

    plt.legend()
    plt.xlabel('Episode')
    plt.ylabel('Steps per episode')
    plt.title('Q-learning ')

    plt.show()

a = {0:-10,1:10,2:20}
show_plot(a)