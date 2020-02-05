import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import rc, animation
import numpy as np
import collections #TODO: redo D from list of tuples to OrderedDict
import time

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

def move(structures, candidates, D, T, pos, dt): #inefficient but whatever
	global pos_orig
	if len(D) == 0:
		return True
	done = [False] * len(candidates)
	for i, m in enumerate(candidates):
		p = T.pred[m].keys()[0]
		diff = tuple(xi - xd for xi, xd in zip(pos[p], pos[m]))
		#limit = 0.22 #for Lattice
		#limit = 0.44 #for Loop
		limit = 0.59 #for Bridges
		#limit = 0.54 #for Hole-in-middle
		if abs(diff[0]) < limit and abs(diff[1]) < limit: #for Loop
			done[i] = True
			continue
		sub_structs = nx.connected_component_subgraphs(structures)
		for s in sub_structs:
			if m in s:
				for n in s.nodes():
					x_p = 0.8
					if round(pos_orig[m][0]) == round(pos_orig[p][0]) and pos[p][0] - pos[m][0] > 0.04: #parent moved right
						if pos[m][1] < pos[p][1]:
							pos[n] = (pos[n][0] + (diff[0] + 1)*dt, pos[n][1] + (diff[1] - x_p)*dt)
						if pos[m][1] > pos[p][1]:
							pos[n] = (pos[n][0] + (diff[0] + 1)*dt, pos[n][1] + (diff[1] + x_p)*dt)
					elif round(pos_orig[m][0]) == round(pos_orig[p][0]) and pos[m][0] > pos[p][0]: #parent moved left
						if pos[m][1] < pos[p][1]:
							pos[n] = (pos[n][0] + (diff[0] - 1)*dt, pos[n][1] + (diff[1] - x_p)*dt)
						if pos[m][1] > pos[p][1]:
							pos[n] = (pos[n][0] + (diff[0] - 1)*dt, pos[n][1] + (diff[1] + x_p)*dt)
					elif round(pos_orig[m][1]) == round(pos_orig[p][1]) and pos[p][1] - pos[m][1] > 0.04: #parent moved up
						if pos[m][0] < pos[p][0]:
							pos[n] = (pos[n][0] + (diff[0] - x_p)*dt, pos[n][1] + (diff[1] + 1)*dt)
						if pos[m][0] > pos[p][0]:
							pos[n] = (pos[n][0] + (diff[0] + x_p)*dt, pos[n][1] + (diff[1] + 1)*dt)
					elif round(pos_orig[m][1]) == round(pos_orig[p][1]) and pos[m][1] > pos[p][1]: #parent moved down
						if pos[m][0] < pos[p][0]:
							pos[n] = (pos[n][0] + (diff[0] - x_p)*dt, pos[n][1] + (diff[1] - 1)*dt)
						if pos[m][0] > pos[p][0]:
							pos[n] = (pos[n][0] + (diff[0] + x_p)*dt, pos[n][1] + (diff[1] - 1)*dt)
					else:
						pos[n] = (pos[n][0] + diff[0]*dt, pos[n][1] + diff[1]*dt)
	return done

def update(i, structures, T, D, M, pos, color_map, ax, dt):
	ax.clear()
	ax.autoscale(enable=False)
	global labels, pos_orig
	#ax.set_xlim([-1.0,3.0]) #for Lattice
	#ax.set_ylim([-1.0,3.0])
	#ax.set_xlim([-6.0,2.0]) #for Loop
	#ax.set_ylim([-1.0,7.0])
	ax.set_xlim([-9.0,2.0]) #for Bridges
	ax.set_ylim([-1.0,10.0])
	#ax.set_xlim([-6.0,4.0]) #for Hole-in-middle
	#ax.set_ylim([-2.0,8.0])
	candidates = [k for k, v in D if v == D[0][1]]
	cand_filter = []
	for c in candidates:
		m_t = labels[c].split("$")[1]
		p = T.pred[c].keys()[0]
		if m_t == labels[p].split("$")[1]:# and p in structures.nodes: 
			continue
		cand_filter.append(c)

	skipped = list(set(candidates) - set(cand_filter))
	reverse = []
	reverse_skip = []
	check = len(cand_filter) > 0
	if check:
		candidates = cand_filter
	for i, c in enumerate(candidates):
		reversal = False
		try:
			m_t = labels[c].split("$")[1]
			p = T.pred[c].keys()[0]
			child = T[c].keys()[0]
			child_child = T[child].keys()[0]
			diff = tuple(xi - xd for xi, xd in zip(pos_orig[p], pos_orig[child]))
			diff_n = tuple(xi - xd for xi, xd in zip(pos[c], pos[child]))
			diff2 = tuple(xi - xd for xi, xd in zip(pos[child], pos[child_child]))
			if m_t > labels[child].split("$")[1] and abs(diff[0]) > 0.7 and abs(diff[1]) > 0.7 and np.sqrt(diff_n[0]**2 + diff_n[1]**2) > 0.59:
				if np.sqrt(diff2[0]**2 + diff2[1]**2) > 0.6:
					continue

				reverse.append(child)
				reverse_skip.append(c)
				reversal = True
				c = child
		except IndexError:
			pass
		new_struct = True
		for n in T[c].keys():
			if n in structures.nodes and reversal != True:
				new_struct = False
				structures.add_edge(c, n)
		if new_struct is True:
			structures.add_node(c)
	candidates.extend(reverse)
	candidates = [n for n in candidates if n not in reverse_skip]
	done = move(structures, candidates, D, T, pos, dt)
	if all(done) is True:
		for c in candidates:
			if T.pred[c].keys()[0] in structures.nodes:
				structures.add_edge(c, T.pred[c].keys()[0])
		try:
			current = D.pop(0)[1]
			while D[0][1] == current:
				m = D.pop(0)
				if m[0] in skipped and check:
					D.append(m)
				if m[0] in reverse_skip and check:
					D.append(m)
			D = sort_by_step(D)
		except IndexError:
			#plt.close()
			time.sleep(200)
		
	nx.draw_networkx(T_plt, pos, with_labels=True, arrows=True, node_color=color_map, ax=ax, node_size=400, node_shape="s")
	#nx.draw_networkx(T_plt, pos, labels=labels, arrows=True, node_color=color_map, ax=ax, node_size=1000, node_shape="s")

	#ax.set_title("Frame {}".format(i))

def animate(T, D, M, pos, color_map):
	fig, ax = plt.subplots(figsize=(7,7))
	dt = 1./30

	structures = nx.Graph()
	structures.add_nodes_from([k for k, v in D if v == D[0][1]])
	anim = animation.FuncAnimation(fig, update, interval=20, fargs=(structures, T, D, M, pos, color_map, ax, dt), save_count=10000)

	try:
		plt.show()
		#anim.save('bridges.mp4', fps=30, extra_args=["-vcodec","libx264"])
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
	'''
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
	'''
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
	#'''
	#Bridges
        G = nx.Graph()
        G.add_nodes_from(list(range(41)))
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
        40: (-8.0, 1.0),
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
        G.add_edge(14,40)
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
	#'''
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
	pos_orig = {k: v for k, v in pos.items()}
	for step in D:
		p = T_plt.pred[step[0]].keys()[0]
		if p == M:
			if step[0] == 29: #for Bridges
				step = (step[0], 0)
			labels[step[0]] = r"${}$".format(step[1])#temporary
			labels[M] = r"$0$"#temporary
			color_map[step[0]] = "red"
		else:
			labels[step[0]] = r"${}$".format(step[1])
			color_map[step[0]] = color_node(step[1])
	animate(T_plt, D, M, pos, color_map)
	#plt.xlabel(r"$x$", fontsize = 14)
	#plt.ylabel(r"$y$", fontsize = 14)
	#nx.draw_networkx(T_plt, pos, labels=labels, arrows=True, node_color=color_map, ax=ax, node_size=1000, node_shape="s")
