import heapq
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict

def dijkstra(graph, start):
    pq = [(0, start)]  # Priority queue with (distance, node)
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parents = {node: None for node in graph}
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if current_distance > distances[current_node]:
            continue
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parents[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
    
    return distances, parents

def visualize_graph(graph, shortest_path=None):
    G = nx.Graph()
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            G.add_edge(node, neighbor, weight=weight)
    
    pos = nx.spring_layout(G)
    labels = {(u, v): G[u][v]['weight'] for u, v in G.edges}
    
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray', node_size=2000, font_size=12)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    
    if shortest_path:
        path_edges = list(zip(shortest_path, shortest_path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)
        nx.draw_networkx_nodes(G, pos, nodelist=shortest_path, node_color='orange', node_size=2000)
    
    plt.show()

def reconstruct_path(parents, start, end):
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = parents[current]
    path.reverse()
    return path if path[0] == start else []

def main():
    graph = defaultdict(dict)
    
    try:
        num_edges = int(input("Enter the number of edges: "))
        if num_edges <= 0:
            print("Number of edges must be positive.")
            return
    except ValueError:
        print("Invalid input! Please enter an integer.")
        return

    print("Enter edges in the format: start_node end_node weight")
    for _ in range(num_edges):
        try:
            u, v, w = input().split()
            w = int(w)
            graph[u][v] = w
            graph[v][u] = w  # Assuming an undirected graph
        except ValueError:
            print("Invalid input! Please enter in the correct format.")

    start_node = input("Enter the start node: ")
    if start_node not in graph:
        print(f"Error: Start node '{start_node}' is not in the graph.")
        return

    distances, parents = dijkstra(graph, start_node)
    print("Shortest distances from", start_node, ":", distances)
    
    end_node = input("Enter the end node to find the shortest path: ")
    if end_node not in graph:
        print(f"Error: End node '{end_node}' is not in the graph.")
        return

    shortest_path = reconstruct_path(parents, start_node, end_node)
    if not shortest_path:
        print(f"No path found between {start_node} and {end_node}.")
    else:
        print(f"Shortest path from {start_node} to {end_node}:", " -> ".join(shortest_path))
    
    visualize_graph(graph, shortest_path)

if __name__ == "__main__":
    main()
