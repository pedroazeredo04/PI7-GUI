from antlr4 import *
from GCode.GCodeLexer import GCodeLexer
from GCode.GCodeParser import GCodeParser
from GCode.GCodeListener import GCodeListener
import numpy as np

class WalkListener(GCodeListener):
    def __init__(self):
        self.coordinates = []
    def enterStatement(self, ctx):
        x = None
        y = None
        if ctx.codfunc().getText() == 'G01' or ctx.codfunc().getText() == 'G00':
            if ctx.coordx() is None and ctx.coordy() is None:
                print("Error in line %s: %s command must have X or Y coordinates" % (ctx.numerolinha().getText(), ctx.codfunc().getText()))
            else:
                if ctx.coordy() is None:
                    y=self.coordinates[-1][1]
                else:
                    y=ctx.coordy().coord().getText()
                if ctx.coordx() is None:
                    x=self.coordinates[-1][0]
                else:
                    x=ctx.coordx().coord().getText()
                self.coordinates.append((x, y))

def parsing(filename):
    with open(filename) as file:
        data = f'{file.read()}'
    lexer = GCodeLexer(InputStream(data))
    stream = CommonTokenStream(lexer)
    parser = GCodeParser(stream)
    tree = parser.gcode()
    listener = WalkListener()
    walker = ParseTreeWalker()
    walker.walk(listener, tree)
    return np.array(listener.coordinates)