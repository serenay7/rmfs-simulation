import networkx as nx
import matplotlib.pyplot as plt
import Entities

def place_shelf(network, start_pos, shelf_dimensions=(2, 4)):
    rows, cols = shelf_dimensions
    for r in range(rows):
        for c in range(cols):
            node = (start_pos[0] + r, start_pos[1] + c)
            if node in network.nodes:
                network.nodes[node]['shelf'] = True
                tempPod = Entities.Pod
                network.nodes[node]['pod'] = tempPod

def draw_network_with_shelves(G, pos):
    # Node colors based on shelf attribute
    node_colors = ['skyblue' if not G.nodes[node].get('shelf') else 'salmon' for node in G.nodes()]

    plt.figure(figsize=(20, 10))
    nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=600, node_color=node_colors, font_color='black')
    plt.show()

def place_shelves_automatically(network, shelf_dimensions=(4, 2), spacing=(1, 1)):
    rows, cols = network.graph['rows'], network.graph['cols']
    shelf_rows, shelf_cols = shelf_dimensions
    space_rows, space_cols = spacing

    # Loops to place shelves wherever possible
    for r in range(1, rows, shelf_rows + space_rows):
        for c in range(1, cols, shelf_cols + space_cols):
            start_pos = (r, c)
            # Shelf placement from specified starting position
            place_shelf(network, start_pos, shelf_dimensions)

def create_rectangular_network_with_attributes(rows, columns):
    G = nx.grid_2d_graph(rows, columns)
    G.graph['rows'] = rows
    G.graph['cols'] = columns
    pos = dict((node, node) for node in G.nodes())
    return G, pos

def create_corridor_subgraph(G):
    """
    Creates a subgraph containing only the corridor nodes (i.e., nodes not part of shelves).
    """
    # Extract nodes that are not part of a shelf
    corridor_nodes = [node for node, data in G.nodes(data=True) if not data.get('shelf', False)]
    return G.subgraph(corridor_nodes).copy()

def create_node_added_subgraph(specific_node, subgraph, graph):
    """
    Creates a subgraph containing an additionaly specific node.
    """
    # Create a shallow copy of the existing subgraph
    node_added_subgraph = subgraph.copy()

    # Add the specific node to the copy of the subgraph
    node_added_subgraph.add_node(specific_node, **graph.nodes[specific_node])

    # Add edges of the specific node to the copy of the subgraph
    for neighbor in graph.neighbors(specific_node):
        if neighbor in node_added_subgraph.nodes:
            node_added_subgraph.add_edge(specific_node, neighbor)

    return node_added_subgraph

if __name__ == "__main__":
    # Creating a new warehouse network and racking
    rows = 4
    columns = 4
    rectangular_network, pos = create_rectangular_network_with_attributes(rows, columns)
    place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    #print(rectangular_network._node)
    #create_subgraph(rectangular_network)
    #rectangular_network[1][1]["shelf"] = True
    #print(rectangular_network[1][1]["shelf"])
    test = create_corridor_subgraph(rectangular_network)
    node_added_subgraph = create_node_added_subgraph((3,1), test, rectangular_network)
    a = rectangular_network.nodes(data=True)
    nx.draw(node_added_subgraph, with_labels=True, node_color='skyblue', node_size=500, edge_color='gray')

    # Display the graph
    plt.show()
    # Visualizing the updated network
    draw_network_with_shelves(rectangular_network, pos)
