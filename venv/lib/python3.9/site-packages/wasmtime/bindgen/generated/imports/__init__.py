from .python import Python
from dataclasses import dataclass

@dataclass
class BindgenImports:
    python: Python
