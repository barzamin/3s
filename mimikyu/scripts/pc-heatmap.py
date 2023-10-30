#!/usr/bin/env python3
import matplotlib.pyplot as plt
import click
import numpy as np
import re

RE_LINE = re.compile(r"([0-9a-f]+) -> ([0-9a-f]+)")


@click.command
@click.argument("tracefile", type=click.File("r"))
def cli(tracefile):
    hits = np.zeros(0xffff+1)

    for line in tracefile:
        if m := RE_LINE.match(line):
            from_pc, _to_pc = m.groups()
            from_pc = int(from_pc, 16)
            hits[from_pc] += 1

    [fig, ax] = plt.subplots()
    axim = ax.imshow(hits[:0x100].reshape(32, -1), norm='log')
    fig.colorbar(axim, extend='max')
    plt.show()

if __name__ == "__main__":
    cli()
