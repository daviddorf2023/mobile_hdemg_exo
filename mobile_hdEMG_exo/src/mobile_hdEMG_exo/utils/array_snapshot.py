from collections.abc import Collection


class CollectionSnapshot:
    _collection: Collection
    _position: int

    def __init__(self, collection: Collection):
        self._collection = collection
        self._position = len(collection)

    def sync(self):
        self._position = len(self._collection)

    def read(self) -> Collection:
        return self._collection[self._position]
