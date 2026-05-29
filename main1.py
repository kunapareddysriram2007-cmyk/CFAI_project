# Complete Corrected Autonomous Cleaning Strategy Planner
import pygame
import heapq
import random
import time
import sys

# =========================================================
# GRID SETTINGS
# =========================================================

GRID_ROWS = 20
GRID_COLS = 20

CELL_SIZE = 35
MARGIN = 2

PANEL_WIDTH = 300

WIN_W = GRID_COLS * (CELL_SIZE + MARGIN) + PANEL_WIDTH
WIN_H = GRID_ROWS * (CELL_SIZE + MARGIN) + 80

FPS = 10

OBSTACLE_RATIO = 0.15
DIRTY_RATIO = 0.12

BATTERY_MAX = 200

# =========================================================
# COLORS
# =========================================================

BG = (20, 22, 30)

GREY = (180, 180, 180)
BLACK = (30, 30, 30)
RED = (200, 50, 50)
GREEN = (50, 180, 90)
YELLOW = (255, 220, 60)
BLUE = (70, 130, 255)

WHITE = (255, 255, 255)

PANEL_BG = (28, 32, 45)

# =========================================================
# CELL TYPES
# =========================================================

EMPTY = 0
OBSTACLE = 1
DIRTY = 2

# =========================================================
# HEURISTIC
# =========================================================

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# =========================================================
# A* SEARCH
# =========================================================

def astar(grid, start, goal):

    rows = len(grid)
    cols = len(grid[0])

    open_heap = []

    heapq.heappush(
        open_heap,
        (heuristic(start, goal), 0, start)
    )

    came_from = {}

    g_score = {start: 0}

    in_open = {start}

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    while open_heap:

        _, current_g, current = heapq.heappop(open_heap)

        in_open.discard(current)

        if current == goal:

            path = []

            node = goal

            while node in came_from:
                path.append(node)
                node = came_from[node]

            path.append(start)

            path.reverse()

            return path

        for dr, dc in directions:

            nr = current[0] + dr
            nc = current[1] + dc

            if 0 <= nr < rows and 0 <= nc < cols:

                if grid[nr][nc] != OBSTACLE:

                    neighbor = (nr, nc)

                    tentative_g = g_score[current] + 1

                    if tentative_g < g_score.get(neighbor, float('inf')):

                        came_from[neighbor] = current

                        g_score[neighbor] = tentative_g

                        f_score = tentative_g + heuristic(neighbor, goal)

                        if neighbor not in in_open:

                            heapq.heappush(
                                open_heap,
                                (f_score, tentative_g, neighbor)
                            )

                            in_open.add(neighbor)

    return []

# =========================================================
# GRID CREATION
# =========================================================

def make_grid():

    grid = []

    for r in range(GRID_ROWS):

        row = []

        for c in range(GRID_COLS):

            value = EMPTY

            if random.random() < OBSTACLE_RATIO:
                value = OBSTACLE

            row.append(value)

        grid.append(row)

    # Safe start area
    for r in range(3):
        for c in range(3):
            grid[r][c] = EMPTY

    # Add dirty cells
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):

            if grid[r][c] == EMPTY:

                if random.random() < DIRTY_RATIO:
                    grid[r][c] = DIRTY

    return grid

# =========================================================
# FIND DIRTY CELLS
# =========================================================

def get_dirty_cells(grid):

    dirty = []

    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):

            if grid[r][c] == DIRTY:
                dirty.append((r, c))

    return dirty

# =========================================================
# CELL RECT
# =========================================================

def cell_rect(r, c):

    x = c * (CELL_SIZE + MARGIN) + MARGIN

    y = r * (CELL_SIZE + MARGIN) + 70

    return pygame.Rect(
        x,
        y,
        CELL_SIZE,
        CELL_SIZE
    )

# =========================================================
# MAIN
# =========================================================

def main():

    pygame.init()

    screen = pygame.display.set_mode((WIN_W, WIN_H))

    pygame.display.set_caption(
        "Autonomous Cleaning Strategy Planner"
    )

    clock = pygame.time.Clock()

    font = pygame.font.SysFont("segoeui", 18)

    grid = make_grid()

    robot = (1, 1)

    charging_station = (1, 1)

    dirty_cells = get_dirty_cells(grid)

    cleaned = set()

    visited = set()

    visited.add(robot)

    skipped = set()

    battery = BATTERY_MAX

    steps = 0

    start_time = time.time()

    current_path = []

    current_target = None

    running = True

    completed = False

    completion_time = 0

    while True:

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # =================================================
        # CLEANING LOGIC
        # =================================================

        if running and battery > 0:

            dirty_cells = [
                cell for cell in dirty_cells
                if cell not in cleaned
                and cell not in skipped
            ]

            if not current_path:

                current_target = None

                candidates = sorted(
                    dirty_cells,
                    key=lambda x: heuristic(robot, x)
                )

                for candidate in candidates:

                    if candidate in cleaned or candidate in skipped:
                        continue

                    path = astar(grid, robot, candidate)

                    if path and len(path) > 1:

                        current_target = candidate

                        current_path = path[1:]

                        break

                    else:

                        skipped.add(candidate)

                if not current_target and not current_path:

                    running = False

                    completed = True

                    completion_time = int(
                        time.time() - start_time
                    )

            if current_path:

                robot = current_path.pop(0)

                visited.add(robot)

                battery -= 1

                steps += 1

                if grid[robot[0]][robot[1]] == DIRTY:

                    cleaned.add(robot)

                    grid[robot[0]][robot[1]] = EMPTY

                if (
                    not current_path
                    and current_target
                    and robot != current_target
                ):

                    path = astar(
                        grid,
                        robot,
                        current_target
                    )

                    if path and len(path) > 1:

                        current_path = path[1:]

                    else:

                        skipped.add(current_target)

                        current_target = None

        # =================================================
        # DRAW
        # =================================================

        screen.fill(BG)

        pygame.draw.rect(
            screen,
            PANEL_BG,
            (0, 0, WIN_W, 60)
        )

        title = font.render(
            "Autonomous Cleaning Strategy Planner",
            True,
            WHITE
        )

        screen.blit(title, (20, 18))

        # Draw grid
        for r in range(GRID_ROWS):

            for c in range(GRID_COLS):

                rect = cell_rect(r, c)

                if grid[r][c] == OBSTACLE:

                    pygame.draw.rect(screen, RED, rect)

                elif grid[r][c] == DIRTY:

                    pygame.draw.rect(screen, BLACK, rect)

                elif (r, c) in visited:

                    pygame.draw.rect(screen, GREEN, rect)

                else:

                    pygame.draw.rect(screen, GREY, rect)

        # Charging station
        cr, cc = charging_station

        pygame.draw.rect(
            screen,
            BLUE,
            cell_rect(cr, cc),
            3
        )

        # Robot
        rr, rc = robot

        pygame.draw.ellipse(
            screen,
            YELLOW,
            cell_rect(rr, rc).inflate(-6, -6)
        )

        # =================================================
        # SIDE PANEL
        # =================================================

        panel_x = GRID_COLS * (CELL_SIZE + MARGIN) + 20

        pygame.draw.rect(
            screen,
            PANEL_BG,
            (
                GRID_COLS * (CELL_SIZE + MARGIN),
                0,
                PANEL_WIDTH,
                WIN_H
            )
        )

        if completed:
            elapsed = completion_time
        else:
            elapsed = int(time.time() - start_time)

        info = [
            f"Battery : {battery}%",
            f"Steps Taken : {steps}",
            f"Time Elapsed : {elapsed}s",
            f"Dirty Cells Left : {len(dirty_cells)}",
            f"Cells Cleaned : {len(cleaned)}",
            f"Algorithm : A* Search"
        ]

        y = 100

        for line in info:

            text = font.render(line, True, WHITE)

            screen.blit(text, (panel_x, y))

            y += 40

        # =================================================
        # LEGEND
        # =================================================

        legend = [
            ("Grey  - Unvisited", GREY),
            ("Black - Dirty Cell", BLACK),
            ("Green - Cleaned", GREEN),
            ("Red   - Obstacle", RED),
            ("Blue  - Charging", BLUE)
        ]

        y += 20

        for text_label, color in legend:

            pygame.draw.rect(
                screen,
                color,
                (panel_x, y, 20, 20)
            )

            text = font.render(
                text_label,
                True,
                WHITE
            )

            screen.blit(text, (panel_x + 30, y - 2))

            y += 35

        # =================================================
        # STATUS MESSAGE
        # =================================================

        if completed:

            pygame.draw.rect(
                screen,
                (40, 120, 60),
                (panel_x, y + 20, 240, 50),
                border_radius=10
            )

            complete_text = font.render(
                "Cleaning Completed!",
                True,
                WHITE
            )

            screen.blit(
                complete_text,
                (panel_x + 20, y + 35)
            )

        elif battery <= 0:

            pygame.draw.rect(
                screen,
                (140, 50, 50),
                (panel_x, y + 20, 240, 50),
                border_radius=10
            )

            battery_text = font.render(
                "Battery Depleted!",
                True,
                WHITE
            )

            screen.blit(
                battery_text,
                (panel_x + 25, y + 35)
            )

        pygame.display.flip()

        clock.tick(FPS)

# =========================================================
# RUN PROGRAM
# =========================================================

if __name__ == "__main__":
    main()
