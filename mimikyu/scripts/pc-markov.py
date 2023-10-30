#!/usr/bin/env python3

import re
import click

# import networkx as nx
import pygraphviz as pgv
import numpy as np

RE_LINE = re.compile(r"([0-9a-f]+) -> ([0-9a-f]+) \| (.*)$")


@click.command
@click.argument("tracefile", type=click.File("r"))
@click.option("-o", "--output", type=click.Path(), default="pc-chain.svg")
def cli(tracefile, output):
    # G = nx.DiGraph()
    G = pgv.AGraph(strict=True, directed=True)
    G.graph_attr["fontname"] = G.node_attr["fontname"] = G.edge_attr[
        "fontname"
    ] = "PragmataPro Mono"
    hits = np.zeros(0xffff+1, np.int64)
    for line in tracefile:
        if m := RE_LINE.match(line):
            from_pc, to_pc, asm = m.groups()
            from_pc = int(from_pc, 16)
            to_pc = int(to_pc, 16)
            hits[from_pc] += 1
            h = hits[from_pc]

            from_pc = f"{from_pc:#06x}"
            to_pc = f"{to_pc:#06x}"

            G.add_edge(from_pc, to_pc)
            G.get_node(from_pc).attr["label"] = f"{from_pc} {asm} ({h} hits)"

    if "0x0000" in G:
        G.get_node("0x0000").attr["color"] = "red"
    G.node_attr["shape"] = "box"

    # import matplotlib.pyplot as plt
    # nx.draw_kamada_kawai(
    #     G, with_labels=True,
    #     labels={pc: f"{pc:#04x}" for pc in G.nodes},
    #     node_color=['red' if pc == 0x0000 else 'blue' for pc in G.nodes],
    #     font_color='grey',
    # )
    # plt.show()

    G.layout("dot")
    G.write("G.dot")
    G.draw(output)


if __name__ == "__main__":
    cli()
