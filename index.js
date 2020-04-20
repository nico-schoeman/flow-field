export function flow_cell(
    x = null,
    y = null,
    walkCost = 0,
    isGoal = false,
    isBuildable = false,
    isWalkable = false
) {
    this.x = x;
    this.y = y;
    this.isGoal = isGoal;
    this.isBuildable = isBuildable;
    this.isWalkable = isWalkable;
    this.walkCost = walkCost;
    this.flowDataToGoal = new WeakMap(); // use set/get
}

flow_cell.prototype.flowCost = function(
    goalCell,
    value = Number.MAX_SAFE_INTEGER,
    reset = false
) {
    value += this.walkCost;
    if (
        !this.flowDataToGoal.has(goalCell) ||
        this.flowDataToGoal.get(goalCell).cost == Number.MAX_SAFE_INTEGER
    ) {
        let flowData = { cost: value };
        this.flowDataToGoal.set(goalCell, flowData);
    } else if (reset || !this.flowDataToGoal.get(goalCell).hasOwnProperty('cost')) {
        let flowData = this.flowDataToGoal.get(goalCell);
        flowData.cost = value;
        this.flowDataToGoal.set(goalCell, flowData);
    }
    return this.flowDataToGoal.get(goalCell);
};

// //TODO Use walk values in flow direction calculation
flow_cell.prototype.flowVector = function(goalCell, x, y, reset) {
    if (!this.flowDataToGoal.has(goalCell)) {
        let flowData = { x: x, y: y };
        this.flowDataToGoal.set(goalCell, flowData);
    } else if (reset) {
        let flowData = this.flowDataToGoal.get(goalCell);
        flowData.x = x;
        flowData.y = y;
        this.flowDataToGoal.set(goalCell, flowData);
    }
    return this.flowDataToGoal.get(goalCell);
};

export default function flow_grid() {
    this.grid = {};
    this.waypoints = [];

    this.grid.sizeX = 10;
    this.grid.sizeY = 10;

    this.grid.cells = new Array(this.totalSize.call(this))
        .fill()
        .map(() => new flow_cell());

    for(let x = 0; x < this.grid.sizeX; x++) {
        for(let y = 0; y < this.grid.sizeY; y++) {
            let cell = this.grid.cells[x + this.grid.sizeX * y];
            cell.x = x;
            cell.y = y;
        }
    };
}

flow_grid.prototype.totalSize = function() {
    return this.grid.sizeX * this.grid.sizeY;
};

flow_grid.prototype.validateCell = function(pointX, pointY) {
    return (
        pointX >= 0 &&
        pointX < this.grid.sizeX &&
        pointY >= 0 &&
        pointY < this.grid.sizeY
    );
};

flow_grid.prototype.getCell = function(x, y) {
    return this.grid.cells[x + this.grid.sizeX * y];
};

flow_grid.prototype.setCell = function(newCell) {
    return (this.grid.cells[x + this.grid.sizeX * y] = newCell);
};

flow_grid.prototype.getCellFromWorldPoint = function (worldX, worldY, cellWidth, cellHeight) {
    let x = Math.floor(worldX / cellWidth);
    let y = Math.floor(worldY / cellHeight);
    return this.getCell(x, y);
};

flow_grid.prototype.getCellNeighbors = function(cell, cross = false) {
    let neighbours = [];
    for (let x = -1; x <= 1; x++) {
        for (let y = -1; y <= 1; y++) {
            if (x == 0 && y == 0) continue;
            if (cross && x != 0 && y != 0) continue;

            let checkX = cell.x + x;
            let checkY = cell.y + y;
            //TODO: how to handle position props?

            if (
                checkX >= 0 &&
                checkX < this.grid.sizeX &&
                checkY >= 0 &&
                checkY < this.grid.sizeY
            )
                neighbours.push(this.getCell(checkX, checkY));
        }
    }

    return neighbours;
};

flow_grid.prototype.floodFillDistanceToGoal = function (goal) {
    this.grid.cells.forEach((cell) => cell.flowCost(goal));

    let checkedCells = [];
    let cellsToCheck = [];
    goal.flowCost(goal, 0);
    cellsToCheck.push(goal);

    while (cellsToCheck.length > 0) {
        let checkCell = cellsToCheck.shift();
        let neighbors = this.getCellNeighbors(checkCell, true);

        neighbors.forEach(neighbor => {
            if (neighbor.flowCost(goal).cost == Number.MAX_SAFE_INTEGER && !cellsToCheck.find((cell) => {return cell === neighbor})) {
                neighbor.flowCost(goal, checkCell.flowCost(goal).cost + 1);
                cellsToCheck.push(neighbor);
            }
        });

        checkedCells.push(checkCell);

        cellsToCheck.sort((a, b) => {
            if (a.walkCost == b.walkCost) return 0;
            else if (a.walkCost < b.walkCost) return -1;
            else if (a.walkCost > b.walkCost) return 1;
            else return a.walkCost.compareTo(b.walkCost);
        });
    }
}

flow_grid.prototype.generateFlowFieldVectors = function (goal) {
    this.grid.cells.forEach (cell => {
        cell.flowVector(goal, 0, 0, true);

        let bestFlowCost = Number.MAX_SAFE_INTEGER;
        let bestNeighbor = null;
        //todo: do not flow to node that shares 1 or more non-walkable neighbors - test
        this.getCellNeighbors(cell).forEach(neighbor => {
            let neighborNeighbors = this.getCellNeighbors(neighbor, true).filter(check => !check.isWalkable);
            let myNeighbors = this.getCellNeighbors(cell, true).filter(check => !check.isWalkable);

            if (!neighborNeighbors.some(check => myNeighbors.includes(check)))
            if (neighbor.flowCost(goal).cost < bestFlowCost) {
                bestFlowCost = neighbor.flowCost(goal).cost;
                bestNeighbor = neighbor;
            }
        });

        let x = bestNeighbor.x - cell.x;
        let y = bestNeighbor.y - cell.y;
        cell.flowVector(goal, x, y, true);
    });
}

flow_grid.prototype.generateFlowField = function (goal) {
    this.floodFillDistanceToGoal(goal);
    this.generateFlowFieldVectors(goal);
}

// public List<T> GetCellsOnLine (Vector2 point1, Vector2 point2) {
//     List<T> cells = new List<T>();

//     bool steep = Mathf.Abs(point2.y - point1.y) > Mathf.Abs(point2.x - point1.x);
//     if (steep) {
//         Swap<float>(ref point1.x, ref point1.y);
//         Swap<float>(ref point2.x, ref point2.y);
//     }
//     if (point1.x > point2.x) {
//         Swap<float>(ref point1.x, ref point2.x);
//         Swap<float>(ref point1.y, ref point2.y);
//     }

//     int dX = ((int)point2.x - (int)point1.x);
//     int dY = Mathf.Abs((int)point2.y - (int)point1.y);
//     int err = (dX / 2);
//     int ystep = ((int)point1.y < (int)point2.y ? 1 : -1);
//     int y = (int)point1.y;

//     for (int x = (int)point1.x; x <= (int)point2.x; ++x) {
//         if (steep) {
//             cells.Add(grid[(int)(y + (gridSizeX * x))]);
//         } else {
//             cells.Add(grid[(int)(x + (gridSizeX * y))]);
//         }
//         err = err - dY;
//         if (err < 0) {
//             y += ystep;
//             err += dX;
//         }
//     }

//     return cells;
// }

// private static void Swap<T>(ref T lhs, ref T rhs) {
//     T temp;
//     temp = lhs;
//     lhs = rhs;
//     rhs = temp;
// }

// public enum Rotations {None, Left, Right, Flip}
// public static T[] Rotate (T[] tempGrid, Rotations rotation, int width, int height) {
//     T[] originalGrid = tempGrid;
//     IEnumerable<T> rotatedGrid;

//     rotatedGrid = originalGrid;
//     if (rotation == Rotations.Flip) {
//         rotatedGrid = originalGrid.Reverse();
//     } else {
//         // Rotate left:
//         IEnumerable<int> firstRowIndeces = Enumerable.Range(0, height).Select(i => i * width).Reverse().ToArray();
//         rotatedGrid = Enumerable.Repeat(firstRowIndeces, width).SelectMany((frpi, rowIndex) => frpi.Select(i => originalGrid[i + rowIndex]));

//         if (rotation == Rotations.Right) {
//             rotatedGrid = rotatedGrid.Reverse();
//         }
//     }

//     //partRect = new Rect(partRect.x, partRect.y, partRect.height, partRect.width);
//     //Texture2D newTexture = new Texture2D(texture.height, texture.width);
//     //newTexture.SetPixels32( rotatedPixels.ToArray() );
//     //newTexture.Apply();
//     //CalculateConnections();
//     return rotatedGrid.ToArray();
// }

// public Grid<PathNode> AssumeValues<T> (Grid<T> gridToUse) where T: new() {
//     Grid<PathNode> newGrid = new Grid<PathNode>(gridToUse.gridSizeX, gridToUse.gridSizeY);
//     newGrid.grid = gridToUse.grid.Cast<PathNode>().ToArray<PathNode>();
//     return newGrid;
// }

///////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

// static int GetDistance(FlowNode nodeA, FlowNode nodeB) {
//     int dstX = (int)Mathf.Abs(nodeA.pos.x - nodeB.pos.x);
//     int dstY = (int)Mathf.Abs(nodeA.pos.y - nodeB.pos.y);

//     if (dstX > dstY) {
//         return 14*dstY + 10* (dstX-dstY);
//     }

//     return 14*dstX + 10 * (dstY-dstX);
// }