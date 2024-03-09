import networkx as nx
import matplotlib.pyplot as plt

def create_rectangular_network(rows, columns):
    G = nx.grid_2d_graph(rows, columns)
    pos = dict((node, node) for node in G.nodes())
    plt.figure(figsize=(20,20))
    nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=350, node_color='skyblue', font_color='black')
    plt.show()
    return G

