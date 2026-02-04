#!/usr/bin/env python3
import yaml
import random
import os

# ================= Configuration =================
# The X here must be consistent with your dual_arm_replay.py (0.42)
BOARD_CENTER_X = 0.42
SPACING = 0.08
PLACE_Z = 0.35 

def get_board_xyz(grid_idx):
    row = grid_idx // 3
    col = grid_idx % 3
    y = (1 - row) * SPACING
    x = BOARD_CENTER_X + (col - 1) * SPACING
    return x, y, PLACE_Z

# ================= Tic-Tac-Toe Brain (Exact replica of tic.py) =================
class TicTacToeBrain:
    def __init__(self):
        self.WIN_PATTERNS = [
            (0, 1, 2), (3, 4, 5), (6, 7, 8), 
            (0, 3, 6), (1, 4, 7), (2, 5, 8), 
            (0, 4, 8), (2, 4, 6)
        ]

    def check_win(self, board, player):
        for a, b, c in self.WIN_PATTERNS:
            if board[a] == board[b] == board[c] == player:
                return True
        return False

    def find_fork_moves(self, board, player):
        """Find Fork moves"""
        fork_moves = []
        for i in [x for x in range(9) if board[x] == ' ']:
            b_copy = board[:]
            b_copy[i] = player
            winning_paths = 0
            for a, b, c in self.WIN_PATTERNS:
                line = [b_copy[a], b_copy[b], b_copy[c]]
                if line.count(player) == 2 and line.count(' ') == 1:
                    winning_paths += 1
            if winning_paths >= 2:
                fork_moves.append(i)
        return fork_moves

    def get_best_move(self, board, my_player):
        """
        High-priority logic fully synchronized with tic.py
        """
        opponent = 'O' if my_player == 'X' else 'X'
        available = [i for i, x in enumerate(board) if x == ' ']
        
        # 1. Win
        for m in available:
            b_copy = board[:]
            b_copy[m] = my_player
            if self.check_win(b_copy, my_player): return m
        
        # 2. Block
        for m in available:
            b_copy = board[:]
            b_copy[m] = opponent
            if self.check_win(b_copy, opponent): return m

        # 3. Create Fork - [Previously missing]
        my_forks = self.find_fork_moves(board, my_player)
        if my_forks:
            return random.choice(my_forks)

        # 4. Block Fork
        opp_forks = self.find_fork_moves(board, opponent)
        if len(opp_forks) == 1:
            return opp_forks[0] # Only one fork move available, block it
        elif len(opp_forks) >= 2:
            # Advanced defense: force opponent to the side to break the fork
            sides = [1, 3, 5, 7]
            playable_sides = [m for m in sides if m in available]
            if playable_sides: return random.choice(playable_sides)
            return random.choice(opp_forks)

        # 5. Take Center
        if 4 in available: return 4

        # 6. Take Opposite Corner - [Previously missing]
        corners_map = {0: 8, 8: 0, 2: 6, 6: 2}
        for c, o in corners_map.items():
            if board[c] == opponent and board[o] == ' ': return o

        # 7. Take Empty Corner - [Previously missing]
        corners = [0, 2, 6, 8]
        open_corners = [m for m in corners if m in available]
        if open_corners: return random.choice(open_corners)

        # 8. Take Empty Side - [Previously missing]
        sides = [1, 3, 5, 7]
        open_sides = [m for m in sides if m in available]
        if open_sides: return random.choice(open_sides)

        # 9. Random fallback
        return random.choice(available)

# ================= Main Generation Logic =================
def generate():
    # Path handling
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_yaml_path = os.path.join(script_dir, 'detected_objects.yaml')
    output_yaml_path = os.path.join(script_dir, 'game_script.yaml')

    if not os.path.exists(input_yaml_path):
        print(f"Error: Cannot find {input_yaml_path}")
        return

    with open(input_yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        red_pool = data.get('red_cubes', [])
        blue_pool = data.get('blue_cylinders', [])

    print(f"Inventory: Red={len(red_pool)}, Blue={len(blue_pool)}")
    
    brain = TicTacToeBrain()
    board = [' '] * 9
    script = []
    
    # === Left Arm (X) starts randomly ===
    current = 'X'
    first_move = random.randint(0, 8)
    next_move = first_move
    
    step_count = 0
    red_idx = 0
    blue_idx = 0
    
    while True:
        step_count += 1
        action = {}
        action['step'] = step_count
        
        place_x, place_y, place_z = get_board_xyz(next_move)
        action['place'] = [float(place_x), float(place_y), float(place_z)]
        action['grid_idx'] = next_move
        
        if current == 'X':
            if not red_pool: break
            action['arm'] = 'left'
            pick_pos = red_pool.pop(0)
            action['pick'] = pick_pos
            action['obj_id'] = f"red_{red_idx+1}"
            red_idx += 1
        else:
            if not blue_pool: break
            action['arm'] = 'right'
            pick_pos = blue_pool.pop(0)
            action['pick'] = pick_pos
            action['obj_id'] = f"blue_{blue_idx+1}"
            blue_idx += 1
            
        script.append(action)
        board[next_move] = current
        
        if brain.check_win(board, current):
            print(f"Simulation Result: {current} Wins!")
            break
        if ' ' not in board:
            print("Simulation Result: Draw")
            break
            
        current = 'O' if current == 'X' else 'X'
        next_move = brain.get_best_move(board, current)

    with open(output_yaml_path, 'w') as f:
        yaml.dump({'moves': script}, f, sort_keys=False)
    
    print(f"Successfully generated high-quality game script: {output_yaml_path}")

if __name__ == "__main__":
    generate()