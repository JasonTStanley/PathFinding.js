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


BubbleStarFinder.prototype.findPath = function(startX, startY, endX, endY, grid) {
    var openList = new Heap(function(a, b) { return a.f - b.f; });
    var closeSet = new Set();
    var bestG = {};
    var bubbles = {};

    var start = {
        center: [startX, startY],
        radius: 0,
        cost: 0,
        parent: null,
        via: null
    };
    var goal = {
        center: [endX, endY],
        radius: 0,
        cost: 0,
        parent: null,
        via: null
    };

    function gridSdfQuery(x, y) {
        if (!grid.isInside(x, y)) {
            // Treat out-of-bounds as obstacle
            return 0;
        }
        if (!grid.isWalkableAt(x, y)) {
            return 0;
        }
        var minDist = Infinity;
        for (var i = 0; i < grid.width; ++i) {
            for (var j = 0; j < grid.height; ++j) {
                if (!grid.isWalkableAt(i, j)) {
                    var dx = x - i, dy = y - j;
                    var dist = Math.sqrt(dx * dx + dy * dy);
                    if (dist < minDist) {
                        minDist = dist;
                    }
                }
            }
        }
        // If no obstacles, return a large value
        return (minDist === Infinity) ? Math.max(grid.width, grid.height) : minDist;
    }
    function heuristic(current) {
        return this.heuristic(Math.abs(current[0] - endX), Math.abs(current[1] - endY));
    }

    function bubbleDistance(a, b) {
        var dx = a[0] - b[0], dy = a[1] - b[1];
        return Math.sqrt(dx*dx + dy*dy);
    }

    function sphereEdge(radius) {
        var points = [];
        var r2 = radius * radius;
        var lo = Math.floor(-radius), hi = Math.ceil(radius + 1);
        for (var x = lo; x < hi; ++x) {
            for (var y = lo; y < hi; ++y) {
                var d2 = x*x + y*y;
                if (d2 >= r2) continue;
                var is_boundary = false;
                for (var dx = -1; dx <= 1; ++dx) {
                    for (var dy = -1; dy <= 1; ++dy) {
                        if (dx === 0 && dy === 0) continue;
                        var nx = x + dx, ny = y + dy;
                        var nd2 = nx*nx + ny*ny;
                        if (nd2 >= r2) {
                            is_boundary = true;
                            break;
                        }
                    }
                    if (is_boundary) break;
                }
                if (is_boundary) points.push([x, y]);
            }
        }
        return points;
    }

    function pushOpen(bubble) {
        var key = gridVecKey(bubble.center[0], bubble.center[1]);
        var g = bubble.cost;
        if (closeSet.has(key)) return;
        if (bestG[key] !== undefined && g >= bestG[key]) return;
        bestG[key] = g;
        bubbles[key] = bubble;
        var f = g + heuristic.call(this, bubble.center);
        bubble.f = f;
        openList.push(bubble);
        // Debug: log bubble opened
        console.log('Opened bubble:', bubble.center, 'cost:', bubble.cost, 'radius:', bubble.radius);
    }

    function popOpen() {
        while (!openList.empty()) {
            var bubble = openList.pop();
            var key = gridVecKey(bubble.center[0], bubble.center[1]);
            if (closeSet.has(key)) continue;
            var retry = false;
            closeSet.forEach(function(closedKey) {
                var closedBubble = bubbles[closedKey];
                if (!closedBubble) return;
                if (bubble.parent && closedKey === gridVecKey(bubble.parent.center[0], bubble.parent.center[1])) return;
                var dist = bubbleDistance(bubble.center, closedBubble.center);
                if (dist < closedBubble.radius - 1.5) retry = true;
            });
            if (retry) continue;
            if (bestG[key] !== undefined && bubble.cost > bestG[key] + 1e-5) continue;
            var pos = [bubble.center[0], bubble.center[1]];
            var sdf = gridSdfQuery(pos[0], pos[1], grid);
            bubble.radius = sdf;
            // Debug: log bubble popped
            console.log('Popped bubble:', bubble.center, 'cost:', bubble.cost, 'radius:', bubble.radius);
            return bubble;
        }
        return null;
    }

    function goalReached(bubble) {
        return bubbleDistance(bubble.center, goal.center) < bubble.radius;
    }

    function gridVecKey(x, y) {
        return x + ',' + y;
    }

    function getNeighbor(bubble) {
        var neighbors = [];
        var r = bubble.radius;
        var edgePoints = sphereEdge(r);
        if (!bubble.parent) {
            for (var i = 0; i < edgePoints.length; ++i) {
                var next = [bubble.center[0] + edgePoints[i][0], bubble.center[1] + edgePoints[i][1]];
                var temp = {
                    center: next,
                    radius: 0,
                    cost: bubble.cost + Math.sqrt(edgePoints[i][0]*edgePoints[i][0] + edgePoints[i][1]*edgePoints[i][1]),
                    parent: bubble,
                    via: bubble
                };
                neighbors.push(temp);
            }
            return neighbors;
        }
        for (var i = 0; i < edgePoints.length; ++i) {
            var next = [bubble.center[0] + edgePoints[i][0], bubble.center[1] + edgePoints[i][1]];
            var temp = {
                center: next,
                radius: 0,
                cost: bubble.cost + Math.sqrt(edgePoints[i][0]*edgePoints[i][0] + edgePoints[i][1]*edgePoints[i][1]),
                parent: bubble,
                via: bubble
            };
            neighbors.push(temp);
        }
        return neighbors;
    }

    var startKey = gridVecKey(start.center[0], start.center[1]);
    bubbles[startKey] = start;
    bestG[startKey] = 0;
    pushOpen.call(this, start);

    while (!openList.empty()) {
        var current = popOpen.call(this);
        if (!current) break;
        var currentKey = gridVecKey(current.center[0], current.center[1]);
        if (goalReached(current)) {
            var path = [];
            var node = current;
            while (node) {
                path.push([node.center[0], node.center[1]]);
                node = node.parent;
            }
            path.push([goal.center[0], goal.center[1]]);
            path.reverse();
            // Debug: log path found
            console.log('Path found:', path);
            return path;
        }
        closeSet.add(currentKey);
        // Visualization: mark closed cell
        var x = current.center[0], y = current.center[1];
        var node = grid.getNodeAt(x, y);
        if (node) node.closed = true;
        // Debug: log closed cell
        console.log('Closed cell:', current.center);
        var neighbors = getNeighbor(current);
        for (var i = 0; i < neighbors.length; ++i) {
            var neighbor = neighbors[i];
            var nKey = gridVecKey(neighbor.center[0], neighbor.center[1]);
            if (closeSet.has(nKey)) continue;
            pushOpen.call(this, neighbor);
        }
    }
    // Debug: log failure
    console.log('No path found');
    return [];
}
module.exports = BubbleStarFinder;
