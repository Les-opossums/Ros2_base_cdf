"""State Machine for pliers."""
from enum import IntEnum, auto


class GlobalSM(IntEnum):
    """Global State Machine of the Pliers system."""

    NOP = auto()
    PICK = auto()
    RELEASE = auto()
    EXPLORE = auto()
    GOHOME = auto()
    STOP = auto()
    FINISH = auto()
    CURSOR = auto()
    GOBOARD = auto()
    FINISHSEQUENCESM = auto()

class GoBoardSM(IntEnum):
    """GoBoard State Machine."""
    GO_BOARD_INIT = auto()
    GO_BOARD_NAV = auto()
    GO_BOARD_DONE = auto()

class CursorSM(IntEnum):
    """Cursor State Machine."""
    CURSOR_INIT = auto()
    CURSOR_NAV = auto()
    CURSOR_APPROACH_DOWN = auto()
    CURSOR_CURSOR = auto()
    CURSOR_LEAVE_UP = auto()
    CURSOR_DONE = auto()
class PickSM(IntEnum):
    """Pick State Machine of the system."""

    PICK_INIT = auto()
    PICK_NAV = auto()
    PICK_STARE = auto()
    PICK_UPDATE = auto()
    PICK_CENTERING = auto()
    PICK_PICK = auto()
    PICK_DONE = auto()
    PICK_FAILED = auto()
    PICK_RETRY = auto()

class ReleaseSM(IntEnum):
    """Release State Machine of the system."""

    RELEASE_INIT = auto()
    RELEASE_NAV = auto()
    RELEASE_LOOK = auto()
    RELEASE_DROP = auto()
    RELEASE_DONE = auto()
    RELEASE_FAILED = auto()

class GoHomeSM(IntEnum):
    """Go Home State Machine of the system."""

    GOHOME_INIT = auto()
    GOHOME_NAV_E_ZONE = auto()
    GOHOME_NAV_FINAL = auto()
    GOHOME_DROP = auto()
    GOHOME_DONE = auto()
    GOHOME_FAILED = auto()

class ExploreSM(IntEnum):
    """Explore State Machine of the system."""

    EXPLORE_INIT = auto()
    EXPLORE_NAV = auto()
    EXPLORE_STARE = auto()
    EXPLORE_UPDATE = auto()
    EXPLORE_DONE = auto()   

class FinishSequenceSM(IntEnum):
    """Finish Sequence State Machine of the system."""

    FINISH_SEQUENCE_INIT = auto()
    FINISH_SEQUENCE_NAV = auto()
    FINISH_SEQUENCE_MOVE = auto()
    FINISH_SEQUENCE_DOWN = auto()
    FINISH_SEQUENCE_DROP = auto()
    FINISH_SEQUENCE_DONE = auto()

class StopSM(IntEnum):
    """Stop State Machine of the system."""

    STOP_INIT = auto()
    STOP_STOP = auto()
    STOP_DONE = auto()

# 1. Map Global States to their corresponding Sub-State Classes
_SUB_STATE_MAP = {
    GlobalSM.EXPLORE: ExploreSM,
    GlobalSM.PICK: PickSM,
    GlobalSM.RELEASE: ReleaseSM,
    GlobalSM.GOHOME: GoHomeSM,
    GlobalSM.FINISHSEQUENCESM: FinishSequenceSM,
}


def resolve_state_names(global_val, sub_val) -> tuple[str, str]:
    """
    Convert raw state values (int or Enum) into human-readable strings.

    Returns: (global_name, sub_state_name)
    """
    try:
        # Ensure we have the Enum object for the Global State
        g_enum = GlobalSM(global_val)
        g_name = g_enum.name
    except ValueError:
        return f"UNKNOWN_GLOBAL({global_val})", str(sub_val)

    # Resolve Sub-State
    sub_enum_class = _SUB_STATE_MAP.get(g_enum)

    if sub_enum_class:
        try:
            # Convert the sub_val int to its specific Enum name
            s_name = sub_enum_class(sub_val).name
        except ValueError:
            s_name = f"UNKNOWN_SUB({sub_val})"
    else:
        # If the global state (like NOP) has no sub-machine
        s_name = "NONE"

    return g_name, s_name
