import matplotlib.pyplot as plt
import random
def show_plot(result,res):
    plt.figure(figsize=(15, 4))
    plt.plot(result.keys(), result.values(),
             )
    plt.plot(res.keys(), res.values(),
             )

    plt.legend(['FSM_Q-learning','Q-learning'])
    plt.xlabel('Episode')
    plt.ylabel('Steps per episode')
    plt.title('result ')

    plt.show()
show_plot(a,b)
# print(res)