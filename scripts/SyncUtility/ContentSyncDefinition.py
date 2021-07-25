from enum import Enum

class ContentSyncType(Enum):
    UNKNOWN=0
    MANUAL=1,
    AUTOMATIC=2,

class FolderType(Enum):
    GENERIC=0,
    CONFIG=1,
    SOURCE=2,
    BINARY=3