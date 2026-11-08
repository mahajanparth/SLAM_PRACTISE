#include <queue>
#include <utility>
#include <vector>

std::vector<std::vector<int>> labelFreeRegions(const std::vector<std::vector<int>>& grid) {
  const int R = static_cast<int>(grid.size());
  const int C = R ? static_cast<int>(grid[0].size()) : 0;
  std::vector<std::vector<int>> label(R, std::vector<int>(C, -1));
  const int dr[4] = {1, -1, 0, 0}, dc[4] = {0, 0, 1, -1};
  int next = 0;
  std::queue<std::pair<int, int>> q;  // explicit queue: no recursion depth problems
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      if (grid[r][c] != 0 || label[r][c] != -1) continue;
      label[r][c] = next;  // label on push so each cell is enqueued once
      q.push({r, c});
      while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop();
        for (int k = 0; k < 4; ++k) {
          const int nr = cr + dr[k], nc = cc + dc[k];
          if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
          if (grid[nr][nc] != 0 || label[nr][nc] != -1) continue;
          label[nr][nc] = next;
          q.push({nr, nc});
        }
      }
      ++next;
    }
  }
  return label;
}

int countFreeRegions(const std::vector<std::vector<int>>& grid) {
  int mx = -1;
  for (const auto& row : labelFreeRegions(grid))
    for (int l : row) mx = l > mx ? l : mx;
  return mx + 1;
}
