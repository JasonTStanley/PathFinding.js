var Heap = require('heap');
var Util = require('../core/Util');
var Heuristic = require('../core/Heuristic');
var DiagonalMovement = require('../core/DiagonalMovement');

/**
 * Bubble* path-finder (Bubble A*).
 * @constructor
 *@param {Object} opt
 * @param {boolean} opt.allowDiagonal Whether diagonal movement is allowed.
 *     Deprecated, use diagonalMovement instead.
 * @param {boolean} opt.dontCrossCorners Disallow diagonal movement touching
 *     block corners. Deprecated, use diagonalMovement instead.
 * @param {DiagonalMovement} opt.diagonalMovement Allowed diagonal movement.
 * @param {function} opt.heuristic Heuristic function to estimate the distance
 *     (defaults to manhattan).
 * @param {number} opt.weight Weight to apply to the heuristic to allow for
 *     suboptimal paths, in order to speed up the search.
 */
function BubbleStarFinder(opt) {
    opt = opt || {};
    this.allowDiagonal = opt.allowDiagonal;
    this.dontCrossCorners = opt.dontCrossCorners;
    this.heuristic = opt.heuristic || Heuristic.euclidean;
    this.weight = opt.weight || 1;
    this.diagonalMovement = opt.diagonalMovement;
    this.debug = !!opt.debug;
    this.onStep = typeof opt.onStep === 'function' ? opt.onStep : null;
    this.debuggerBreak = !!opt.debuggerBreak;
    this.maxIterations = opt.maxIterations || 50000;

    if (!this.diagonalMovement) {
        if (!this.allowDiagonal) {
            this.diagonalMovement = DiagonalMovement.Never;
        } else {
            if (this.dontCrossCorners) {
                this.diagonalMovement = DiagonalMovement.OnlyWhenNoObstacles;
            } else {
                this.diagonalMovement = DiagonalMovement.IfAtMostOneObstacle;
            }
        }
    }

    // When diagonal movement is allowed the manhattan heuristic is not
    //admissible. It should be octile instead
    if (this.diagonalMovement === DiagonalMovement.Never) {
        this.heuristic = opt.heuristic || Heuristic.manhattan;
    } else {
        this.heuristic = opt.heuristic || Heuristic.octile;
    }
}

BubbleStarFinder.prototype._buildOccupiedCellList = function(grid) {
    var occupied = [];
    var x;
    var y;

    for (x = 0; x < grid.width; ++x) {
        for (y = 0; y < grid.height; ++y) {
            if (!grid.isWalkableAt(x, y)) {
                occupied.push([x, y]);
            }
        }
    }

    return occupied;
};

BubbleStarFinder.prototype.signedDistanceAt = function(x, y, grid, occupiedCells) {
    var nearest;
    var i;
    var dx;
    var dy;
    var dist;

    if (!grid.isInside(x, y)) {
        return 0;
    }

    occupiedCells = occupiedCells || this._buildOccupiedCellList(grid);

    if (!occupiedCells.length) {
        nearest = Math.max(grid.width, grid.height);
    } else {
        nearest = Infinity;
        for (i = 0; i < occupiedCells.length; ++i) {
            dx = x - occupiedCells[i][0];
            dy = y - occupiedCells[i][1];
            dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < nearest) {
                nearest = dist;
            }
        }
    }


    return nearest;
};


BubbleStarFinder.prototype.findPath = function(startX, startY, endX, endY, grid) {
    var openList = new Heap(function(nodeA, nodeB) {
            return nodeA.f - nodeB.f;
        }),
        startNode = grid.getNodeAt(startX, startY),
        endNode = grid.getNodeAt(endX, endY),
        heuristic = this.heuristic,
        diagonalMovement = this.diagonalMovement,
        weight = this.weight,
        abs = Math.abs, SQRT2 = Math.SQRT2,
        node, neighbors, neighbor, i, l, x, y, ng;

    
    occupiedCells = this._buildOccupiedCellList(grid);

    // set the `g` and `f` value of the start node to be 0
    startNode.g = 0;
    startNode.f = 0;


    // push the start node into the open list
    openList.push(startNode);
    startNode.opened = true;

    


    // while the open list is not empty
    while (!openList.empty()) {
        // pop the position of node which has the minimum `f` value.
        node = openList.pop();
        node.closed = true;

        // if reached the end position, construct the path and return it
        if (node === endNode) {
            return Util.backtrace(endNode);
        }

        // get neigbours of the current node
        neighbors = grid.getNeighbors(node, diagonalMovement);
        for (i = 0, l = neighbors.length; i < l; ++i) {
            neighbor = neighbors[i];

            if (neighbor.closed) {
                continue;
            }

            x = neighbor.x;
            y = neighbor.y;

            // get the distance between current node and the neighbor
            // and calculate the next g score
            ng = node.g + ((x - node.x === 0 || y - node.y === 0) ? 1 : SQRT2);

            // check if the neighbor has not been inspected yet, or
            // can be reached with smaller cost from the current node
            if (!neighbor.opened || ng < neighbor.g) {
                neighbor.g = ng;
                neighbor.h = neighbor.h || weight * heuristic(abs(x - endX), abs(y - endY));
                neighbor.f = neighbor.g + neighbor.h;
                neighbor.parent = node;

                if (!neighbor.opened) {
                    openList.push(neighbor);
                    neighbor.opened = true;
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    openList.updateItem(neighbor);
                }
            }
        } // end for each neighbor
    } // end while not open list empty

    // fail to find the path
    return [];
};

module.exports = BubbleStarFinder;
