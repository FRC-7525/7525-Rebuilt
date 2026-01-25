from matplotlib.patches import FancyArrowPatch

def generate_graph(state_map):
    G = nx.DiGraph()

    for state, info in state_map.items():
        G.add_node(state)
        for c in info.get("connectionsTo", []):
            G.add_edge(state, c["to"], label=c["condition"])

    # Layout
    pos = nx.spring_layout(G, seed=42, k=1.0)
    if "IDLE" in pos:
        pos["IDLE"] = (0, 0)

    # Figure
    fig, ax = plt.subplots(figsize=(20, 20))
    ax.set_facecolor("#f9fafb")
    ax.axis("off")

    # ---- Nodes ----
    idle_nodes = ["IDLE"] if "IDLE" in G else []
    normal_nodes = [n for n in G.nodes if n != "IDLE"]

    nx.draw_networkx_nodes(
        G, pos,
        nodelist=normal_nodes,
        node_size=2800,
        node_color="#e0f2ff",
        edgecolors="#2563eb",
        linewidths=2,
        ax=ax
    )

    nx.draw_networkx_nodes(
        G, pos,
        nodelist=idle_nodes,
        node_size=3600,
        node_color="#ffedd5",
        edgecolors="#c2410c",
        linewidths=3,
        ax=ax
    )

    nx.draw_networkx_labels(
        G, pos,
        font_size=11,
        font_weight="bold",
        font_family="DejaVu Sans",
        ax=ax
    )

    # ---- Edges (manual arrows) ----
    for (src, dst, data) in G.edges(data=True):
        x1, y1 = pos[src]
        x2, y2 = pos[dst]

        arrow = FancyArrowPatch(
            (x1, y1),
            (x2, y2),
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=1.6,
            color="#475569",
            connectionstyle="arc3,rad=0.15",
            zorder=1
        )
        ax.add_patch(arrow)

        # Edge label
        if "label" in data:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(
                mx, my,
                data["label"],
                fontsize=8,
                color="#1f2937",
                ha="center",
                va="center",
                bbox=dict(
                    boxstyle="round,pad=0.25",
                    fc="white",
                    ec="none",
                    alpha=0.9
                ),
                zorder=3
            )

    plt.title(
        "State Machine",
        fontsize=22,
        fontweight="bold",
        pad=24
    )

    plt.tight_layout()
    plt.savefig("state_machine.png", dpi=300)
    plt.show()
