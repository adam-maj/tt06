from abc import abstractmethod
from typing import Protocol

class Python(Protocol):
    @abstractmethod
    def print(self, slice: bytes) -> None:
        raise NotImplementedError
    @abstractmethod
    def eprint(self, slice: bytes) -> None:
        raise NotImplementedError

