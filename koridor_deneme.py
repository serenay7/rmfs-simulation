import networkx as nx
import matplotlib.pyplot as plt


def create_rectangular_network(rows, columns):
    G = nx.grid_2d_graph(rows, columns)
    pos = dict((node, node) for node in G.nodes())
    return G, pos


def place_shelf(network, start_pos, shelf_dimensions=(2, 4)):
    rows, cols = shelf_dimensions
    for r in range(rows):
        for c in range(cols):
            node = (start_pos[0] + r, start_pos[1] + c)
            if node in network.nodes:
                network.nodes[node]['shelf'] = True


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

    # Mümkün olan her yere raf yerleştirmek için döngüler
    for r in range(1, rows, shelf_rows + space_rows):
        for c in range(1, cols, shelf_cols + space_cols):
            start_pos = (r, c)
            # Belirlenen başlangıç pozisyonundan itibaren raf yerleştirme
            place_shelf(network, start_pos, shelf_dimensions)


def create_rectangular_network_with_attributes(rows, columns):
    G = nx.grid_2d_graph(rows, columns)
    G.graph['rows'] = rows  # Ekstra bilgileri depolama
    G.graph['cols'] = columns
    pos = dict((node, node) for node in G.nodes())
    return G, pos

if __name__ == "__main__":
    # Yeni depo ağı oluşturma ve raf yerleştirme
    rows = 5
    columns = 5
    rectangular_network, pos = create_rectangular_network_with_attributes(rows, columns)
    place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    print(rectangular_network.nodes())
    # Güncellenmiş ağı görselleştirme
    draw_network_with_shelves(rectangular_network, pos)
