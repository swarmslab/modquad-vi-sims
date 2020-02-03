import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import rc, animation
import numpy as np
import collections

rc("text", usetex = True)

def select_master():
	b = nx.betweenness_centrality(G)
	b = {k: v for k,v in b.iteritems() if G.degree[k] < 4}
	b_argmax = [k for k,v in b.iteritems() if v == max(b.values())]
	degs = [d for n,d in G.degree if d < 4]
	b_argmax = [i for i in b_argmax if G.degree[i] == max(degs)]
	return b_argmax[len(b_argmax) // 2]

def sort_constrained(c, G, visited, nodes):
	sort = {}
	for n in nodes:
		sort[n] = len(G[n]) - len([i for i in G.neighbors(n) if i in visited or i in c])
	return sorted(sort, key = sort.get)

def constrained_bfs(G, source):
	visited = {source}
	sorted_neighbors = sort_constrained([], G, visited, [n for n in G.neighbors(source)])
	queue = collections.deque([(source, sorted_neighbors)])
	while queue:
		parent, children = queue[0]
		try:
			child = children.pop(0)
			if child not in visited:
				yield parent, child
				visited.add(child)
				queue.append((child, sort_constrained(children,
									G, 
									visited,
									[n for n in G.neighbors(child) if n not in children])))
		except IndexError:
			queue.popleft()

def assembly_tree(G, master):
	#return nx.bfs_tree(G, master)
	T = nx.DiGraph()
	T.add_node(master)
	edges = []
	for e in constrained_bfs(G, master):
		edges.append(e)
	T.add_edges_from(edges)
	return T

def sequence(T, step):
	if len(T) > 1:
		leaves = [i for i in T.nodes if T.out_degree(i) == 0 
						and T.in_degree(i) <= 1]
		D = []
		master = [i for i in T.nodes if T.in_degree(i) == 0][0]
		leaves_size = len(leaves) + 1 if T.out_degree(master) == 1 else len(leaves)
		if leaves_size == 0: #temporary hack
			for l in T.nodes:
				D.append((l, step))
			return D

		limit = int(np.floor(len(T)/leaves_size))
		for l in leaves:
			S = construct_candidates(T, l, limit)
			#print("construct_candidates: {}".format(S.nodes))
			v_M = None
			for n in S.nodes:
				if S.out_degree(n) == 0:
					D.append((n, step))
					T.remove_node(n)
					v_M = n
				else:
					T.remove_node(n)
			S.remove_node(v_M)
			if len(S.nodes) > 0:
				D.append(sequence(S, step + 1))
		if len(T) > 0: #if not all nodes were taken up by construct_candidates
			D.append(sequence(T, step + 1))#TODO: not assume there's only one sub-structure
		return D
	else:
		return (list(T.nodes)[0], step)

def construct_candidates(T, l, limit):
	Si = nx.DiGraph()
	Si.add_node(l)
	try:
		p = [i for i in T.pred[l]][0]
	except IndexError:
		return Si
	while len(Si.nodes) < limit:
		Si.add_node(p)
		Si.add_edge(l, p)
		l = p
		try:
			p = [i for i in T.pred[l]][0]
		except IndexError:
			break
	return Si

def flatten_remove(D, M):
	result = []
	for l in D:
		if isinstance(l, list):
			result += flatten_remove(l, M)
		else:
			if isinstance(l, tuple):
				if(l[0] == M):
					continue
				result.append(l)
	return result

def sort_by_step(D):
	return sorted(D, key = lambda dis: dis[1], reverse = True)

def move(D, T, pos, dt): #inefficient but whatever
	if len(D) == 0:
		return True
	high = D[0][1] #max step
	candidates = [(k,v) for k, v in D if v == high]
	for k, v in candidates:
		p = [i for i in T.pred[k]][0]
		diff = tuple(xi - xd for xi, xd in zip(pos[k], pos[p]))
		pos[k] = (pos[k][0] - diff[0]*dt, pos[k][1] - diff[1]*dt)
		if abs(diff[0]) < 0.21 and abs(diff[1]) < 0.115:
			return True
	return False

def update(i, T, D, M, pos, color_map, ax, dt):
	ax.clear()

	done = move(D, T, pos, dt)
	if done is True:
		try:
			current = D.pop(0)[1]
			while D[0][1] == current:
				D.pop(0)
		except IndexError:
			plt.close()
		
	nx.draw_networkx(T_plt, pos, with_labels=False, arrows=True, node_color=color_map, ax=ax, node_size=1000, node_shape="s")

	ax.set_title("Frame {}".format(i))

def animate(T, D, M, pos, color_map):
	fig, ax = plt.subplots(figsize=(7,7))
	dt = 1./30

	anim = animation.FuncAnimation(fig, update, interval=20, fargs=(T, D, M, pos, color_map, ax, dt), save_count=300)

	try:
		plt.show()
		#anim.save('animation.mp4', fps=30, extra_args=["-vcodec","libx264"])
	except AttributeError:
		pass

def color_node(n):
	mapping = {
		1: 'coral',
		2: 'lightblue',
		3: 'orange',
		4: 'lightgreen',
		5: 'pink',
		6: 'magenta',
		7: 'cyan',
		8: 'olive',
		9: 'yellow' 
	}
	return mapping.get(n, 'lightgray')

if __name__ == "__main__":
	#'''
	#Lattice
	G = nx.Graph()
	G.add_nodes_from(list(range(9)))
	pos = {
	0: (0.0, 0.0),
	1: (0.0, 1.0),
	2: (0.0, 2.0),
	3: (1.0, 0.0),
	4: (1.0, 1.0),
	5: (1.0, 2.0),
	6: (2.0, 0.0),
	7: (2.0, 1.0),
	8: (2.0, 2.0)
	#9: (1.0, 3.0)
	}
	G.add_edge(0,1)
	G.add_edge(1,2)
	G.add_edge(0,3)
	G.add_edge(3,4)
	G.add_edge(4,5)
	G.add_edge(5,8)
	G.add_edge(6,7)
	G.add_edge(7,8)
	G.add_edge(7,4)
	G.add_edge(6,3)
	G.add_edge(4,1)
	G.add_edge(5,2)
	#G.add_edge(5,9)
	#'''
	'''
	#Loop
	G = nx.path_graph(24)
        pos = {
        0: (0.0, 0.0),
        1: (0.0, 1.0),
        2: (0.0, 2.0),
        3: (-1.0, 2.0),
        4: (-1.0, 3.0),
        5: (-1.0, 4.0),
        6: (0.0, 4.0),
        7: (0.0, 5.0),
        8: (0.0, 6.0),
        9: (-1.0, 6.0),
        10: (-2.0, 6.0),
        11: (-3.0, 6.0),
        12: (-4.0, 6.0),
        13: (-4.0, 5.0),
        14: (-4.0, 4.0),
        15: (-3.0, 4.0),
        16: (-3.0, 3.0),
        17: (-3.0, 2.0),
        18: (-4.0, 2.0),
        19: (-4.0, 1.0),
        20: (-4.0, 0.0),
        21: (-3.0, 0.0),
        22: (-2.0, 0.0),
        23: (-1.0, 0.0),
        }
	G.add_edge(23,0)
	'''
	'''
	#Bridges
        G = nx.Graph()
        G.add_nodes_from(list(range(40)))
        pos = {
        0: (0.0, 0.0),
        1: (-3.0, 0.0),
        2: (-4.0, 0.0),
        3: (0.0, 1.0),
        4: (-3.0, 1.0),
        5: (-4.0, 1.0),
        6: (0.0, 2.0),
        7: (-1.0, 2.0),
        8: (-2.0, 2.0),
        9: (-3.0, 2.0),
        10: (-4.0, 2.0),
        11: (-5.0, 2.0),
        12: (-6.0, 2.0),
        13: (-7.0, 2.0),
        14: (-8.0, 2.0),
        15: (0.0, 3.0),
        16: (-1.0, 3.0),
        17: (-2.0, 3.0),
        18: (-3.0, 3.0),
        19: (-4.0, 3.0),
        20: (-5.0, 3.0),
        21: (-6.0, 3.0),
        22: (-7.0, 3.0),
        23: (-8.0, 3.0),
        24: (-3.0, 4.0),
        25: (-4.0, 4.0),
        32: (-1.0, 5.0),
        27: (-2.0, 5.0),
        28: (-3.0, 5.0),
        29: (-4.0, 5.0),
        30: (0.0, 6.0),
        31: (-1.0, 6.0),
        26: (-2.0, 6.0),
        33: (0.0, 7.0),
        34: (-1.0, 7.0),
        #35: (0.0, 8.0),
        #36: (-1.0, 8.0),
        #37: (-2.0, 8.0),
        #38: (-1.0, 9.0),
        #39: (-2.0, 9.0),
        38: (0.0, 8.0),
        36: (-1.0, 8.0),
        37: (-2.0, 8.0),
        35: (-1.0, 9.0),
        39: (-2.0, 9.0),
        }
        G.add_edge(0,3)
        G.add_edge(1,4)
        G.add_edge(1,2)
        G.add_edge(4,5)
        G.add_edge(2,5)
        G.add_edge(6,3)
        G.add_edge(9,4)
        G.add_edge(10,5)
        G.add_edge(6,7)
        G.add_edge(8,7)
        G.add_edge(8,9)
        G.add_edge(10,9)
        G.add_edge(10,11)
        G.add_edge(12,11)
        G.add_edge(12,13)
        G.add_edge(14,13)
        G.add_edge(6,15)
        G.add_edge(7,16)
        G.add_edge(8,17)
        G.add_edge(9,18)
        G.add_edge(10,19)
        G.add_edge(11,20)
        G.add_edge(12,21)
        G.add_edge(13,22)
        G.add_edge(14,23)
        G.add_edge(16,15)
        G.add_edge(17,16)
        G.add_edge(18,17)
        G.add_edge(19,18)
        G.add_edge(20,19)
        G.add_edge(21,20)
        G.add_edge(22,21)
        G.add_edge(23,22)
        G.add_edge(18,24)
        G.add_edge(19,25)
        G.add_edge(28,24)
        G.add_edge(29,25)
        G.add_edge(24,25)
        G.add_edge(29,28)
        G.add_edge(28,27)
        G.add_edge(32,27)
        G.add_edge(32,31)
        G.add_edge(27,26)
        G.add_edge(30,31)
        G.add_edge(31,26)
        G.add_edge(30,33)
        G.add_edge(31,34)
        G.add_edge(33,34)
        G.add_edge(33,38)
        G.add_edge(34,36)
        G.add_edge(38,36)
        G.add_edge(37,36)
        G.add_edge(35,36)
        G.add_edge(39,37)
        G.add_edge(39,35)
	'''
	'''
	#Hole-in-middle
        G = nx.Graph()
        G.add_nodes_from(list(range(41)))
        pos = {
        0: (0.0, 0.0),
        1: (-1.0, 0.0),
        2: (-2.0, 0.0),
        3: (-1.0, 1.0),
        4: (-1.0, 2.0),
        5: (-1.0, 3.0),
        6: (0.0, 3.0),
        7: (1.0, 3.0),
        8: (2.0, 3.0),
        9: (-2.0, 3.0),
        10: (-3.0, 3.0),
        11: (-4.0, 3.0),
        12: (3.0, 4.0),
        13: (2.0, 4.0),
        14: (1.0, 4.0),
        15: (0.0, 4.0),
        16: (-1.0, 4.0),
        17: (-2.0, 4.0),
        18: (-3.0, 4.0),
        19: (-4.0, 4.0),
        20: (-5.0, 4.0),
        21: (-4.0, 5.0),
        22: (-5.0, 5.0),
        33: (2.0, 5.0),
        24: (3.0, 5.0),
        25: (-4.0, 6.0),
        26: (-5.0, 6.0),
        27: (-3.0, 6.0),
        28: (-2.0, 6.0),
        29: (-1.0, 6.0),
        30: (0.0, 6.0),
        31: (1.0, 6.0),
        32: (2.0, 6.0),
        23: (3.0, 6.0),
        34: (0.0, 7.0),
        35: (-1.0, 7.0),
        36: (-2.0, 7.0),
        37: (-3.0, 7.0),
        38: (-4.0, 7.0),
        39: (1.0, 7.0),
        40: (2.0, 7.0),
        }
        G.add_edge(0,1)
        G.add_edge(2,1)
        G.add_edge(3,1)
        G.add_edge(3,4)
        G.add_edge(5,4)
        G.add_edge(5,6)
        G.add_edge(7,6)
        G.add_edge(7,8)
        G.add_edge(5,9)
        G.add_edge(10,9)
        G.add_edge(10,11)
        G.add_edge(19,11)
        G.add_edge(18,10)
        G.add_edge(17,9)
        G.add_edge(13,8)
        G.add_edge(14,7)
        G.add_edge(15,6)
        G.add_edge(16,5)
        G.add_edge(12,13)
        G.add_edge(14,13)
        G.add_edge(14,15)
        G.add_edge(16,15)
        G.add_edge(16,17)
        G.add_edge(18,17)
        G.add_edge(18,19)
        G.add_edge(20,19)
        G.add_edge(19,21)
        G.add_edge(22,20)
        G.add_edge(12,24)
        G.add_edge(13,33)
        G.add_edge(21,25)
        G.add_edge(22,26)
        G.add_edge(21,22)
        G.add_edge(33,24)
        G.add_edge(33,32)
        G.add_edge(24,23)
        G.add_edge(25,26)
        G.add_edge(25,27)
        G.add_edge(28,27)
        G.add_edge(28,29)
        G.add_edge(30,29)
        G.add_edge(30,31)
        G.add_edge(32,31)
        G.add_edge(32,23)
        G.add_edge(30,34)
        G.add_edge(35,34)
        G.add_edge(35,36)
        G.add_edge(37,36)
        G.add_edge(37,38)
        G.add_edge(39,34)
        G.add_edge(39,40)
        G.add_edge(32,40)
        G.add_edge(31,39)
        G.add_edge(35,29)
        G.add_edge(36,28)
        G.add_edge(38,25)
        G.add_edge(37,27)
	'''

	M = select_master() #for Hole-in-middle scenario, choose master 12
	print("Master: {}".format(M))
	T = assembly_tree(G, M)
	T_plt = T.copy()
	D = sequence(T, 1)
	D = flatten_remove(D, M)
	D = sort_by_step(D)
	print("Disassembly Sequence: {}".format(D))
	labels = {}
	color_map = [0] * len(G)
	color_map[M] = 'lightgray'
	for step in D:
		p = [i for i in T_plt.pred[step[0]]][0]
		if p == M:
			color_map[step[0]] = "red"
		else:
			labels[step[0]] = r"${}$".format(step[1])
			color_map[step[0]] = color_node(step[1])
	animate(T_plt, D, M, pos, color_map)
	#plt.xlabel(r"$x$", fontsize = 14)
	#plt.ylabel(r"$y$", fontsize = 14)
	#nx.draw_networkx(T_plt, pos, labels=labels, arrows=True, node_color=color_map, ax=ax, node_size=1000, node_shape="s")
