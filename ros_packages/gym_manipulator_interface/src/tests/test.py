import random as rn




if __name__ == '__main__':
	action_queue = []
	action_space = {0:'y', 
								1:'h',
								2:'u',
								3:'j',
								4:'i',
								5:'k',
								6:'o',
								7:'l',
								8:'g',
								9:'f'}

	for i in range(0, 6):
		action_queue.append(action_space.get(rn.sample(action_space, 1)[0]))

	print(action_queue)
	pop = action_queue.pop()
	print(pop)
