#!/usr/bin/env python3

import networkx as nx
from networkx.algorithms import tree
from networkx.algorithms.euler import eulerian_path
from scipy.signal import convolve2d

import numpy as np
import matplotlib.pyplot as plt

import random
import math
import sys

def distance(pos, i, j):
	dx = pos[i][0] - pos[j][0]
	dy = pos[i][1] - pos[j][1]
	return math.sqrt(dx*dx + dy*dy)

def f_dist(p_prime, P):
	costs = []
	for (M_w, M_d, _, _, _) in p_prime:
		if len(M_d) == 1:
			costs.append(P.subgraph(M_w.flatten()).size(weight = "weight"))
		elif len(M_w) == 1:
			costs.append(P.subgraph(M_d.flatten()).size(weight = "weight"))
		else:
			costs.append(np.absolute(P.subgraph(M_d.flatten()).size(weight = "weight") - P.subgraph(M_w.flatten()).size(weight = "weight")))
	return [m for _, m in sorted(zip(costs, p_prime))]

def f_time(p):
	return sorted(p, key = lambda m: np.absolute(np.count_nonzero(m[0]) - np.count_nonzero(m[1])))

def minimum_matching(MST, G, odd_vert):
	while odd_vert:
		v = odd_vert.pop()
		weight = float("inf")
		closest = 0
		for u in odd_vert:
			if G[v][u]['weight'] < weight:
				weight = G[v][u]['weight']
				closest = u
		MST.add_edge(v, closest, weight = weight)
		odd_vert.remove(closest)

def christofides(G, pos):
	tsp_G = nx.Graph()
	nodes = []
	MST = tree.minimum_spanning_tree(G)
	odd_vert = []
	for i in MST.nodes():
		if MST.degree(i) % 2 != 0: 
			odd_vert.append(i)
	minimum_matching(MST, G, odd_vert) #adds minimum weight matching edges to MST
	start = 1
	visited = [False] * len(MST.nodes())
	curr = start
	visited[curr] = True
	for nd in MST.neighbors(curr):
		if visited[nd] == False or nd == start:
			next = nd
			break
	while next != start:
		visited[next]=True
		tsp_G.add_edge(curr,next,weight = G[curr][next]['weight'])
		curr = next
		for nd in MST.neighbors(curr):
			if visited[nd] == False:
				next = nd
				break
		if next == curr:
			for nd in G.neighbors(curr):
				if visited[nd] == False:
					next = nd
					break
		if next == curr:
			next = start
	tsp_G.add_edge(curr, next, weight = G[curr][next]['weight'])
	return tsp_G

def hamiltonian_path(G, pos):
	P = christofides(G, pos)
	longest = sorted(P.edges.data("weight"), key=lambda t: t[2], reverse=True)[0]
	P.remove_edge(*longest[:2])
	sourceend = [node for (node, deg) in P.degree() if deg == 1]
	source, end = sourceend
	mat_path = np.array(next(nx.all_simple_paths(P, source, end)), dtype=np.int8)
	#print(P.size(weight = "weight"))
	return P, mat_path

def create_graph(n):
	G = nx.empty_graph(n)
	V = range(1, n + 1)

	#random.seed(49)#for Lattice distance-based
	random.seed(137)
	pos = {i: (random.randint(0, 500), random.randint(0, 500)) for i in V}
	for u in V:
		for v in V:
			if u == v:
				continue
			wt = distance(pos, u, v)
			G.add_edge(u, v, weight = wt)

	return G, pos

def reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M, M_mat, M_plus, M_plus_mat, C):
	if M_mat.size == 1 or not np.any(M_mat):
		return
	i += 1
	check_pair(G, T, M_virts, M[1], M[2], direc, M, M_mat, M_plus, M_plus_mat, C, O, i, M[3])

def check_pair(G, T, M_virts, M_w, M_d, direc, M, M_mat, M_plus, M_plus_mat, C, O, i, master_mod):
	M_fit_d, M_fit_w, coords_d, coords_w = reconfigurable(M_d, M_w, M_plus_mat, master_mod, M_d[4], pos_S)
	if M_fit_d == True and M_fit_w == True:
		C[M] = (i, M_d, M_w, coords_d, coords_w)
	else:
		M_virts.append(M_d[3])
		if M_fit_d == True:
			C[M] = (i, M_d, None, coords_d, None)
			reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M_w, M_w_mat, M_plus, M_plus_mat, C)
		elif M_fit_w == True:
			C[M] = (i, None, M_w, None, coords_w)
			reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M_d, M_d_mat, M_plus, M_plus_mat, C)
		else:
			C[M] = (i, None, None, None, None)
			reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M_d, M_d_mat, M_plus, M_plus_mat, C)
			reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M_w, M_w_mat, M_plus, M_plus_mat, C)
	return

def reconfigurable(M_d, M_w, M_plus, master_mod, pos_S):
	allowed_d = convolve2d(M_d, M_plus, mode = "valid")
	allowed_w = convolve2d(M_w, M_plus, mode = "valid")
	coords_d = np.stack(np.where(allowed_d >= 0)).T
	coords_w = np.stack(np.where(allowed_w >= 0)).T
	if coords_d == coords_w:
		return np.size(coords_d) > 0, False, coords_d, None
	return np.size(coords_d) > 0, np.size(coords_w) > 0, coords_d, coords_w

def generate_splits(mat, master):
	rows, cols = mat.shape
	splits = []
	for r in range(1, rows):
		#dock direction for docking meta module - 
		#1: bottom-to-top
		#-1: top-to-bottom
		#2: right-to-left
		#-2: left-to-right
		#m[0] = waiting meta module
		#m[1] = docking meta module
		#m[2] = row/column abutting master meta module
		#m[3] = direction
		top, bottom = mat[:r, :], mat[r:, :]
		if np.any(top == master):
			#splits.append((top, bottom, bottom[0, :], 1))
			splits.append((top, bottom, bottom[0, :], "top"))
		else:
			#splits.append((bottom, top, top[-1, :], 2))
			splits.append((bottom, top, top[-1, :], "bottom"))
	for c in range(1, cols):
		print(mat)
		left, right = mat[:, :c], mat[:, c:]
		if np.any(left == master):
			#splits.append((left, right, right[:, 0], 3))
			splits.append((left, right, right[:, 0], "left"))
		else:
			#splits.append((right, left, left[:, -1], 4))
			splits.append((right, left, left[:, -1], "right"))
	return splits

def generate_path_splits(mat, master):
	try:
		rows, cols = mat.shape
	except ValueError:
		rows, cols = 0, mat.shape[0]
	splits = []
	if rows > 1:
		for r in range(1, rows):
			#dock direction for docking meta module - 
			#1: bottom-to-top
			#-1: top-to-bottom
			#2: right-to-left
			#-2: left-to-right
			#m[0] = waiting meta module
			#m[1] = docking meta module
			#m[2] = row/column abutting master meta module
			#m[3] = direction
			top, bottom = mat[:r, :], mat[r:, :]
			if np.any(top == master):
				#splits.append((top, bottom, bottom[0, :], 1))
				splits.append((top, bottom, bottom[0, 0], top[-1, 0], "top"))
			else:
				#splits.append((bottom, top, top[-1, :], 2))
				splits.append((bottom, top, top[-1, -1], bottom[0, -1], "bottom"))
	else:
		for c in range(1, cols):
			left, right = mat[:, :c], mat[:, c:]
			if np.any(left == master):
				#splits.append((left, right, right[:, 0], 3))
				splits.append((left, right, right[0], left[0][0], "left"))
			else:
				#splits.append((right, left, left[:, -1], 4))
				splits.append((right, left, left[-1], right[0][0], "right"))
	return splits

def select_master(G):
	b = nx.betweenness_centrality(G)
	b = {k: v for k,v in b.items() if G.degree[k] < 4}
	b_argmax = [k for k,v in b.items() if v == max(b.values())]
	degs = [d for n,d in G.degree if d < 4]
	b_argmax = [i for i in b_argmax if G.degree[i] == max(degs)]
	return b_argmax[len(b_argmax) // 2]

def valid_division(M_d, M_w, p_S, master_mod):
	for S_M in p_S:
		if M_w.size == S_M[0].size and M_d.size == S_M[1].size:
			return True, S_M
	return False, None

def sort_by_step(D):
	#return sorted(D.items(), key = lambda dis: dis[1][2], reverse = True) #For Distance based
	return sorted(D.items(), key = lambda dis: dis[1][1], reverse = True) #For Time based

def TimeSequence(G, pos, shape):
	mat_des = np.zeros(shape)
	for k, v in pos.items():
		mat_des[v[::-1]] = k
	mat_des = np.flipud(mat_des)
	print("\nDesired structure: \n{}".format(mat_des))
	master_mod = select_master(G)
	O = {master_mod: ("top", 1)} #module, direction, disassembly sequence num
	C = {}
	print("Master module: {}".format(master_mod))
	TSequence(G, mat_des, C, O, 1, master_mod)
	O = sort_by_step(O)
	print("Time Sequence: {}".format(O))
	return O
	
def TSequence(G, M, C, O, i, master_mod):
	if M.size == 1 or not np.any(M):
		return
	i += 1
	p = generate_splits(M, master_mod)
	p = f_time(p)
	M_w, M_d, cands, direc = p[0]
	try:
		M_virt = select_master(G.subgraph(cands.astype(int)))
	except IndexError:
		return
	O[M_virt] = (direc, i)
	#print("M_d: ", M_d, " M_w: ", M_w, " M_virt: ", M_virt)
	TSequence(G.subgraph(M_d.flatten()), M_d, C, O, i, M_virt)
	TSequence(G.subgraph(M_w.flatten()), M_w, C, O, i, master_mod)
	return

def DSequence(G, M, C, O, i, master_mod):
	if M.size == 1 or not np.any(M):
		return
	i += 1
	p_prime = generate_path_splits(M, master_mod)
	p_prime = f_dist(p_prime, G)
	M_w, M_d, cands, m_w, direc = p_prime[0]
	try:
		M_virt = select_master(G.subgraph(cands.astype(int)))
	except IndexError:
		return
	O[M_virt] = (direc, m_w, i)
	DSequence(G.subgraph(M_d.flatten()), M_d, C, O, i, M_virt)
	DSequence(G.subgraph(M_w.flatten()), M_w, C, O, i, master_mod)
	return

def DistanceSequence(G, pos, S, pos_S, shape):
	P, mat_path = hamiltonian_path(G, pos)
	nx.draw(P, pos, with_labels=True)
	plt.show()
	mat_des = np.zeros(shape)
	for k, v in pos_S.items():
		mat_des[v[::-1]] = k
	print("\nDesired structure: \n{}".format(mat_des))
	mat_des = np.flipud(mat_des)
	master_mod = select_master(P)
	print("Master module: {}".format(master_mod))
	neigh = [n for n in P.neighbors(master_mod)]
	weight = float("inf")
	new_master = master_mod
	for n in neigh:
		w = P.edges[n, master_mod]["weight"]
		if w < weight:
			w = weight
			new_master = n
	master_mod = new_master
	T = nx.DiGraph()
	T.add_node(master_mod)
	O = {master_mod: ("top", -1, 1)} #module, module to dock to (waiting module), direction, disassembly sequence num
	C = {}
	print("Corrected Master module: {}".format(master_mod))
	mat_path = mat_path.reshape(shape)
	print("\nrestructured path: \n{}".format(mat_path))
	DSequence(P, mat_path, C, O, 1, master_mod)
	O = sort_by_step(O)
	print("Distance Sequence: {}".format(O))
	#DSequence(S, mat_des, P, mat_path, C, O, 1, master_mod)
	nx.draw(P, pos, with_labels=True)
	plt.show()
	return O

if __name__ == "__main__":
	#Lattice TSequence
	shape = [3,3]
	G = nx.grid_graph(shape)
	pos = {3 * i + j + 1: (i, j) for i, j in G.nodes()}
	G = nx.relabel_nodes(G, lambda t: 3 * t[0] + t[1] + 1)
	TimeSequence(G, pos, shape)
	#nx.draw(G_des, pos=pos_des, with_labels=True)
	#plt.show()
	#Lattice DSequence
	'''n = 24
	G, pos = create_graph(n)
	shape = [4,6]
	S = nx.grid_graph(shape)
	pos_S = {4 * i + j + 1: (i, j) for i, j in S.nodes()}
	S = nx.relabel_nodes(S, lambda t: 4 * t[0] + t[1] + 1)
	DistanceSequence(G, pos, S, pos_S, shape)'''
	'''G = nx.Graph()
	G.add_nodes_from(list(range(42)))
	pos = {
	41: (0, 0),
	1: (-1, 0),
	2: (-2, 0),
	3: (-1, 1),
	4: (-1, 2),
	5: (-1, 3),
	6: (0, 3),
	7: (1, 3),
	8: (2, 3),
	9: (-2, 3),
	10: (-3, 3),
	11: (-4, 3),
	12: (3, 4),
	13: (2, 4),
	14: (1, 4),
	15: (0, 4),
	16: (-1, 4),
	17: (-2, 4),
	18: (-3, 4),
	19: (-4, 4),
	20: (-5, 4),
	21: (-4, 5),
	22: (-5, 5),
	33: (2, 5),
	24: (3, 5),
	25: (-4, 6),
	26: (-5, 6),
	27: (-3, 6),
	28: (-2, 6),
	29: (-1, 6),
	30: (0, 6),
	31: (1, 6),
	32: (2, 6),
	23: (3, 6),
	34: (0, 7),
	35: (-1, 7),
	36: (-2, 7),
	37: (-3, 7),
	38: (-4, 7),
	39: (1, 7),
	40: (2, 7),
	}
	G.add_edge(41,1)
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
	shape = [8,9]
	TimeSequence(G, pos, shape)'''#Time-sequence hole-in-middle

	#reconfigure Lattice
	'''G = nx.grid_graph(shape)
	pos = {3 * i + j + 1: (i, j) for i, j in G.nodes()}
	G = nx.relabel_nodes(G, lambda t: 3 * t[0] + t[1] + 1)
	S = nx.Graph()
	T = nx.Graph()
	M_virts, i = [], 0
	S.add_nodes_from(list(range(9)))
	pos_S = {
	0: (0.0, 0.0),
	1: (1.0, 0.0),
	2: (2.0, 0.0),
	3: (-1.0, 1.0),
	4: (0.0, 1.0),
	5: (3.0, 0.0),
	6: (-1.0, 2.0),
	7: (0.0, 2.0),
	8: (3.0, 1.0)
	}
	S.add_edge(0,1)
	S.add_edge(1,2)
	S.add_edge(0,3)
	S.add_edge(3,4)
	S.add_edge(4,5)
	S.add_edge(5,8)
	S.add_edge(6,7)
	S.add_edge(7,8)
	S.add_edge(7,4)
	S.add_edge(6,3)
	S.add_edge(4,1)
	S.add_edge(5,2)
	reconfigure_disassembly(i, M_virts, G, T, pos, S, pos_S, M, M_mat, M_plus, M_plus_mat)'''
