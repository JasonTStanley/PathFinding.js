var Heap = require("heap");
var Util = require("../core/Util");
var Heuristic = require("../core/Heuristic");
var DiagonalMovement = require("../core/DiagonalMovement");

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
    this.onStep = typeof opt.onStep === "function" ? opt.onStep : null;
    this.debuggerBreak = !!opt.debuggerBreak;
    this.maxIterations = opt.maxIterations || 50000;
    this.connect = !!opt.connect;

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

BubbleStarFinder.prototype.signedDistanceAt = function(
    x,
    y,
    grid,
    occupiedCells
) {
    var nearest;
    var i;
    var cx;
    var cy;
    var qx;
    var qy;
    var outside;
    var inside;
    var dist;
    var h = 0.5;

    if (!grid.isInside(x, y)) {
        return 0;
    }

    occupiedCells = occupiedCells || this._buildOccupiedCellList(grid);

    // distance to map boundary, if you want to keep treating outside-grid as obstacle
    // TODO: this has an error i think, if we are against the boundary the sdf value should be 1/2.
    // also we should maybe compute the sdf exactly for completeness.
    nearest = Math.min(
        Math.min(x + 1, grid.width - x),
        Math.min(y + 1, grid.height - y)
    );

    for (i = 0; i < occupiedCells.length; ++i) {
        cx = occupiedCells[i][0];
        cy = occupiedCells[i][1];

        qx = Math.abs(x - cx) - h;
        qy = Math.abs(y - cy) - h;

        outside = Math.sqrt(
            Math.max(qx, 0) * Math.max(qx, 0) + Math.max(qy, 0) * Math.max(qy, 0)
        );
        inside = Math.min(Math.max(qx, qy), 0);

        dist = outside + inside;

        if (dist < nearest) {
            nearest = dist;
        }
    }

    return nearest;
};

function canMoveDiagonally(diagonalMovement) {
    return diagonalMovement !== DiagonalMovement.Never;
}

function key(node) {
    if (typeof node === "object") {
        return node.x + "," + node.y;
    }
    return node + "," + arguments[1];
}

function diskBoundaryOffsets(radius, consider_diagonal) {
    var out = [];
    if (radius <= 0) return out;

    var R2 = radius * radius;

    // 8-neighborhood
    var N;
    if (consider_diagonal) {
        N = [
            [1, 0],
            [-1, 0],
            [0, 1],
            [0, -1],
            [1, 1],
            [-1, 1],
            [1, -1],
            [-1, -1],
        ];
    } else {
        N = [
            [1, 0],
            [-1, 0],
            [0, 1],
            [0, -1],
        ];
    }

    // range(-r, r+1) with hi exclusive
    var lo = -radius;
    var hi = radius + 1;

    for (var x = lo; x < hi; x++) {
        for (var y = lo; y < hi; y++) {
            var p2 = x * x + y * y;

            // inside test: strictly inside
            if (p2 > R2) continue;
            if (x === 0 && y === 0) continue;

            // boundary test: any 8-neighbor outside (or on) the circle
            var isBoundary = false;
            for (var i = 0; i < N.length; i++) {
                var nx = x + N[i][0];
                var ny = y + N[i][1];
                var q2 = nx * nx + ny * ny;

                if (q2 > R2) {
                    isBoundary = true;
                    break;
                }
            }

            if (!isBoundary) continue;
            out.push([x, y]);
        }
    }

    return out;
}

function cellsWithinRadius(nodeMap, qx, qy, r) {
    var r2 = r * r;
    var out = [];
    for (var dx = -r; dx <= r; dx++) {
        for (var dy = -r; dy <= r; dy++) {
            if (dx * dx + dy * dy >= r2) continue; // keep circle, exclusive of boundary
            var n = nodeMap.get(key(qx + dx, qy + dy));
            if (n) out.push(n);
        }
    }
    return out;
}

function Bubble(x, y, radius) {
    this.x = x;
    this.y = y;
    this.radius = radius;
}

function bubbleContains(bubble, node) {
    var dx = node.x - bubble.x;
    var dy = node.y - bubble.y;
    var distance_sq = dx * dx + dy * dy;
    return distance_sq <= bubble.radius * bubble.radius;
}

function checkIntersects(node, neighbor) {
    // Implementation for checking intersection between a node and its neighbor
    // Only for Bi-directional: track which boundary (start vs end) sees this neighbor, for meeting-in-the-middle detection
    if (neighbor.by && neighbor.by != node.by) {
        console.log(
            "Neighbor",
            neighbor.x,
            neighbor.y,
            "already opened by",
            neighbor.by,
            "at bubble idx",
            neighbor.bubble_idx,
            "now also seen by",
            node.by,
            "at bubble idx",
            bubble_idx
        );
        intersection = { bubble: bubbles[bubble_idx] };
        return intersection;
    }
    return null;
}

BubbleStarFinder.prototype.estimateHeuristic = function(goalX, goalY, x, y) {
    return this.weight * this.heuristic(Math.abs(x - goalX), Math.abs(y - goalY));
};

/**
 * Compute neighbors for Bubble* expansion from a node.
 *
 * @param {number} goalX        Goal x (for heuristic)
 * @param {number} goalY        Goal y (for heuristic)
 * @param {Object} node          Current node {x,y,cost,parent}
 * @param {number} radius        Radius in world units OR grid units depending on resolution
 * @param {number} bubble_idx    Index of the bubble being expanded (for book-keeping)
 *
 * @returns {Array<Object>} neighbors nodes (new objects) with {x,y,cost,parent}
 */
BubbleStarFinder.prototype.expandAndUpdateBoundary = function(
    goalX,
    goalY,
    grid,
    nodeMap,
    bubbles,
    node,
    radius,
    bubble_idx
) {
    var neighbors = [];
    var considerDiagonal = canMoveDiagonally(this.diagonalMovement);
    var estimateHeuristic = this.estimateHeuristic.bind(this, goalX, goalY);
    var i;
    var j;
    var dx;
    var dy;
    var nx;
    var ny;
    var stepCost;
    var intersection;

    if (radius < 0.5) return { neighbors: neighbors };

    // "sphereEdge" in 2D => your disk boundary offsets for integer radius r
    var edge = diskBoundaryOffsets(radius, considerDiagonal); // returns Array<[dx,dy]>

    // Base case: no parent
    if (!node.parent) {
        for (i = 0; i < edge.length; i++) {
            dx = edge[i][0];
            dy = edge[i][1];
            var nx0 = node.x + dx;
            var ny0 = node.y + dy;
            if (!grid.isInside(nx0, ny0) || !grid.isWalkableAt(nx0, ny0)) {
                continue;
            }
            stepCost = Math.hypot(dx, dy);
            var neighbor = grid.getNodeAt(nx0, ny0);

            // Only for Bi-directional: track which boundary (start vs end) sees this neighbor, for meeting-in-the-middle detection
            if (neighbor.by && neighbor.by != node.by) {
                console.log(
                    "Neighbor",
                    neighbor.x,
                    neighbor.y,
                    "already opened by",
                    neighbor.by,
                    "at bubble idx",
                    neighbor.bubble_idx,
                    "now also seen by",
                    node.by,
                    "at bubble idx",
                    bubble_idx
                );
                intersection = { bubble: bubbles[bubble_idx] };
                break;
            }

            neighbor.g = node.g + stepCost;
            neighbor.h = neighbor.h || estimateHeuristic(nx0, ny0);
            neighbor.f = neighbor.g + neighbor.h;
            neighbor.parent = node;
            neighbor.bubble_idx = bubble_idx;
            neighbors.push(neighbor);
        }
        nodeMap.delete(key(node));
        return { neighbors: neighbors, intersection: intersection };
    }

    // Candidate "via" nodes near current node — use OPEN set within r
    var viaNodes = cellsWithinRadius(nodeMap, node.x, node.y, radius);
    viaNodes.push(node); // also consider the current node as a via candidate
    var K = viaNodes.length;

    if (viaNodes.length > 0) {
        // Precompute bubbles referenced by via nodes (and guard bubble_idx)
        var viaBubbles = [];
        var seenBubbleIdx = {};
        for (i = 0; i < viaNodes.length; i++) {
            var via = viaNodes[i];
            via.closed = true;
            //nodeMap.delete(key(via))

            var idx = via.bubble_idx;
            if (idx != null && bubbles[idx] && !seenBubbleIdx[idx]) {
                seenBubbleIdx[idx] = true;
                viaBubbles.push(bubbles[idx]);
            }
        }

        // Filter edge once
        var filteredEdge = [];
        for (i = 0; i < edge.length; i++) {
            dx = edge[i][0];
            dy = edge[i][1];
            var nx = node.x + dx;
            var ny = node.y + dy;
            var insideViaBubble = false;

            for (j = 0; j < viaBubbles.length; j++) {
                var b = viaBubbles[j];
                var bx = nx - b.x;
                var by = ny - b.y;
                var b_rad = b.radius;
                if (bx * bx + by * by < b_rad * b_rad) {
                    insideViaBubble = true;
                    break;
                }
            }

            if (!insideViaBubble) {
                filteredEdge.push(edge[i]);
            }
        }
        edge = filteredEdge;
    }

    var N = edge.length;

    if (N === 0) {
        console.warn(
            "Bubble* warning: all edge neighbors were inside via bubbles, skipping this bubble",
            radius,
            node.x,
            node.y
        );
        return { neighbors: neighbors };
    }

    // For each edge step, choose best via: min_j (via.cost + dist(via.pos, next))
    for (i = 0; i < N; i++) {
        dx = edge[i][0];
        dy = edge[i][1];
        nx = node.x + dx;
        ny = node.y + dy;
        if (!grid.isInside(nx, ny) || !grid.isWalkableAt(nx, ny)) {
            continue;
        }

        var bestVia = viaNodes[0];
        var bestCost = Infinity;

        for (j = 0; j < K; j++) {
            var v = viaNodes[j];
            var dist = Math.hypot(v.x - nx, v.y - ny);
            var total = v.g + dist;

            if (total < bestCost) {
                bestCost = total;
                bestVia = v;
            }
        }

        var neighbor = grid.getNodeAt(nx, ny);

        // Only for Bi-directional: track which boundary (start vs end) sees this neighbor, for meeting-in-the-middle detection
        if (neighbor.by && neighbor.by != node.by) {
            console.log(
                "Neighbor",
                neighbor.x,
                neighbor.y,
                "already opened by",
                neighbor.by,
                "at bubble idx",
                neighbor.bubble_idx,
                "now also seen by",
                node.by,
                "at bubble idx",
                bubble_idx
            );

            intersection = { bubble: bubbles[bubble_idx] };
            break;
        }

        if (!neighbor.opened || bestCost < neighbor.g) {
            // check if we have a better cost and update the neighbor
            neighbor.g = bestCost;
            neighbor.h = estimateHeuristic(nx, ny);
            neighbor.f = neighbor.g + neighbor.h;
            neighbor.parent = bestVia;
            neighbor.bubble_idx = bubble_idx;
            neighbors.push(neighbor);
        }
    }

    return { neighbors: neighbors, intersection: intersection };
};

BubbleStarFinder.prototype.findConnection = function(
    intersection,
    startNodeMap,
    endNodeMap,
    startNode,
    endNode
) {
    var bubble = intersection.bubble;

    if (!bubble) {
        return null;
    }

    // Candidate "via" nodes near current node — use OPEN set within r
    var viaNodesStart = cellsWithinRadius(
        startNodeMap,
        bubble.x,
        bubble.y,
        bubble.radius
    );
    var viaNodesEnd = cellsWithinRadius(
        endNodeMap,
        bubble.x,
        bubble.y,
        bubble.radius
    );

    // Case 0: if the end node is reachable through the bubble.
    if (bubbleContains(bubble, endNode)) {
        console.log(
            "Case 0: End node is within latest bubble"
        );
        viaNodesEnd.push(endNode); // consider the end node as a via candidate
    }

    // Case 1: if the start node is reachable through the bubble.
    if (bubbleContains(bubble, startNode)) {
        console.log(
            "Case 1: Start node is within the latest bubble"
        );
        viaNodesStart.push(startNode); // consider the start node as a via candidate
    }

    // // Case 2: if the intersection is the case where the end node is reachable through the start node's bubble.
    // if (!startNodeMap.has(key(endNode)) && !endNodeMap.has(key(startNode))) {
    //     console.log("Base case 2: bubbles intersect without containing one another's node");
    //     var tmp = viaNodesStart;
    //     viaNodesStart = viaNodesEnd;
    //     viaNodesEnd = tmp;
    // }

    var bestViaStart, bestViaEnd;
    var bestCost = Infinity;
    for (var j = 0; j < viaNodesStart.length; j++) {
        for (var k = 0; k < viaNodesEnd.length; k++) {
            var viaStart = viaNodesStart[j];
            var viaEnd = viaNodesEnd[k];

            if (viaStart.x === viaEnd.x && viaStart.y === viaEnd.y) {
                console.log("same node via found at bubble idx", viaStart.bubble_idx);
            }

            var dist = Math.hypot(viaStart.x - viaEnd.x, viaStart.y - viaEnd.y);
            var total = viaStart.g + viaEnd.g + dist;

            if (total < bestCost) {
                bestCost = total;
                bestViaStart = viaStart;
                bestViaEnd = viaEnd;
            }
        }
    }
    return { viaStart: bestViaStart, viaEnd: bestViaEnd };
};

BubbleStarFinder.prototype.findPathOneDirection = function(
    startX,
    startY,
    endX,
    endY,
    grid
) {
    var openList = new Heap(function(nodeA, nodeB) {
        return nodeA.f - nodeB.f;
    }),
        nodeMap = new Map(),
        bubbles = [],
        startNode = grid.getNodeAt(startX, startY),
        endNode = grid.getNodeAt(endX, endY),
        expandAndUpdateBoundary = this.expandAndUpdateBoundary.bind(
            this,
            endX,
            endY,
            grid,
            nodeMap,
            bubbles
        ),
        node,
        i;

    var occupiedCells = this._buildOccupiedCellList(grid);

    // set the `g` and `f` value of the start node to be 0
    startNode.g = 0;
    startNode.f = 0;

    // push the start node into the open list
    openList.push(startNode);
    startNode.opened = true;
    nodeMap.set(key(startNode), startNode);

    // while the open list is not empty
    while (!openList.empty()) {
        // pop the position of node which has the minimum `f` value.
        node = openList.pop();
        if (node === endNode) {
            var tmp = node;
            while (tmp.parent) {
                console.log(
                    "Path node:",
                    tmp.x,
                    tmp.y,
                    "via bubble idx",
                    tmp.bubble_idx
                );
                tmp = tmp.parent;
            }
            return Util.backtrace(endNode);
        }

        if (node.closed) {
            // lazy deletion: skip nodes already closed
            continue;
        }
        node.closed = true;

        var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
        radius = Math.floor(radius);
        console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
        var bubble = new Bubble(node.x, node.y, radius);
        bubbles.push(bubble);

        // if reached the end position, construct the path and return it
        if (bubbleContains(bubble, endNode)) {
            console.log("End node is within bubble, connecting directly to end node");
            // calculate the path to the end node,
            var viaNodes = cellsWithinRadius(nodeMap, node.x, node.y, radius);
            viaNodes.push(node); // also consider the current node as a via candidate
            var bestCost = Infinity;
            for (i = 0; i < viaNodes.length; i++) {
                var via = viaNodes[i];
                var dist = Math.hypot(via.x - endNode.x, via.y - endNode.y);
                var total = via.g + dist;
                if (total < bestCost) {
                    bestCost = total;
                    endNode.parent = via;
                    endNode.g = total;
                    endNode.h = 0;
                    endNode.f = total;
                    endNode.bubble_idx = bubbles.length - 1;
                }
            }
            endNode.opened = true;
            openList.push(endNode);
            nodeMap.set(key(endNode), endNode); // TODO: added because Bi-directional also has this, but not needed
        }

        // get neigbours of the current node
        neighbors = expandAndUpdateBoundary(
            node,
            radius,
            bubbles.length - 1
        ).neighbors;
        for (i = 0; i < neighbors.length; ++i) {
            neighbor = neighbors[i];

            if (neighbor.closed) {
                continue;
            }

            x = neighbor.x;
            y = neighbor.y;
            if (!neighbor.opened) {
                openList.push(neighbor);
                neighbor.opened = true;
                nodeMap.set(key(neighbor), neighbor);
            } else {
                // the neighbor can be reached with smaller cost.
                // Since its f value has been updated, we have to
                // update its position in the open list
                openList.updateItem(neighbor);
            }
        } // end for each neighbor
        //nodeMap.delete(key(node))
    } // end while not open list empty

    // fail to find the path
    return [];
};

BubbleStarFinder.prototype.findPathConnect = function (startX, startY, endX, endY, grid) {
    var cmp = function(a, b) {
        return a.f - b.f;
    };
    var startOpenList = new Heap(cmp);
    var endOpenList = new Heap(cmp);
    var startNodeMap = new Map();
    var endNodeMap = new Map();
    var startBubbles = [];
    var endBubbles = [];
    var startNode = grid.getNodeAt(startX, startY);
    var endNode = grid.getNodeAt(endX, endY);
    var BY_START = 1,
        BY_END = 2;
    var node, i;

    var occupiedCells = this._buildOccupiedCellList(grid);

    // set the `g` and `f` value of the start & end nodes to be 0
    startNode.g = 0;
    startNode.f = 0;
    endNode.g = 0;
    endNode.f = 0;

    // push the start node into the start open list
    startOpenList.push(startNode);
    startNode.opened = true;
    startNode.by = BY_START;
    startNodeMap.set(key(startNode), startNode);

    // push the end node into the end open list
    endOpenList.push(endNode);
    endNode.opened = true;
    endNode.by = BY_END;
    endNodeMap.set(key(endNode), endNode);

    // while the open lists are not empty
    while (!startOpenList.empty() && !endOpenList.empty()) {
        // START EXPANSION
        // pop the position of node which has the minimum `f` value.
        node = startOpenList.pop();

        // lazy deletion: skip nodes already closed
        if (!node.closed) {
            node.closed = true;

            var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
            radius = Math.floor(radius);
            console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
            var bubble = new Bubble(node.x, node.y, radius);
            startBubbles.push(bubble);

            // Base case 0: if the end node is reachable through the start node's bubble.
            // if (!node.parent && bubbleContains(bubble, endNode)) {
            //     console.log(
            //         "Base case 0: End node is within 1st Start bubble, connecting directly to end node"
            //     );
            //     // calculate the path to the end node,
            //     total = startNode.g + Math.hypot(node.x - endNode.x, node.y - endNode.y);
            //     endNode.parent = startNode;
            //     endNode.g = total;
            //     endNode.h = 0;
            //     endNode.f = total;
            //     endNode.bubble_idx = startBubbles.length - 1;
            //     endNode.opened = true;
            //     startOpenList.push(endNode);
            //     startNodeMap.set(key(endNode), endNode);
            //     return Util.backtrace(endNode);
            // }

            // get neigbours of the current node
            results = this.expandAndUpdateBoundary(
                endX,
                endY,
                grid,
                startNodeMap,
                startBubbles,
                node,
                radius,
                startBubbles.length - 1
            );
            neighbors = results.neighbors;
            for (i = 0; i < neighbors.length; ++i) {
                neighbor = neighbors[i];

                if (neighbor.closed) {
                    continue;
                }

                x = neighbor.x;
                y = neighbor.y;
                if (!neighbor.opened) {
                    startOpenList.push(neighbor);
                    neighbor.opened = true;
                    neighbor.by = BY_START;
                    startNodeMap.set(key(neighbor), neighbor);
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    startOpenList.updateItem(neighbor);
                }
            } // end for each neighbor

            // Check for meeting in the middle by seeing if any neighbor is already opened by the other search direction
            if (results.intersection) {
                console.log("Meeting in the middle detected! (BY_START)");
                var connection = this.findConnection(
                    results.intersection,
                    startNodeMap,
                    endNodeMap,
                    startNode,
                    endNode
                );
                if (connection) {
                    var path = Util.biBacktrace(connection.viaStart, connection.viaEnd);
                    // Print the path for debugging
                    for (i = 0; i < path.length; i++) {
                        console.log("Path node:", path[i][0], path[i][1]);
                    }
                    return path;
                }
                console.warn(
                    "Connection failed to find a valid via node pair, continuing search"
                );
            }
        }

        // END EXPANSION
        // pop the position of node which has the minimum `f` value.
        node = endOpenList.pop();

        // lazy deletion: skip nodes already closed
        if (!node.closed) {
            node.closed = true;

            var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
            radius = Math.floor(radius);
            console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
            var bubble = new Bubble(node.x, node.y, radius);
            endBubbles.push(bubble);

            // Base case 1: if the start node is reachable through the end node's bubble.
            // if (!node.parent && bubbleContains(bubble, startNode)) {
            //     console.log(
            //         "Base case 1: Start node is within 1st End bubble, connecting directly to start node"
            //     );
            //     // calculate the path to the end node,
            //     total = endNode.g + Math.hypot(node.x - endNode.x, node.y - endNode.y);
            //     startNode.parent = endNode;
            //     startNode.g = total;
            //     startNode.h = 0;
            //     startNode.f = total;
            //     startNode.bubble_idx = endBubbles.length - 1;
            //     startNode.opened = true;
            //     startOpenList.push(endNode);
            //     startNodeMap.set(key(endNode), endNode);
            //     return Util.backtrace(startNode);
            // }

            // get neigbours of the current node
            results = this.expandAndUpdateBoundary(
                startX,
                startY,
                grid,
                endNodeMap,
                endBubbles,
                node,
                radius,
                endBubbles.length - 1
            );
            neighbors = results.neighbors;
            for (i = 0; i < neighbors.length; ++i) {
                neighbor = neighbors[i];

                if (neighbor.closed) {
                    continue;
                }

                x = neighbor.x;
                y = neighbor.y;
                if (!neighbor.opened) {
                    endOpenList.push(neighbor);
                    neighbor.opened = true;
                    neighbor.by = BY_END;
                    endNodeMap.set(key(neighbor), neighbor);
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    endOpenList.updateItem(neighbor);
                }
            } // end for each neighbor

            // Check for meeting in the middle by seeing if any neighbor is already opened by the other search direction
            if (results.intersection) {
                console.log("Meeting in the middle detected! (BY_END)");
                var connection = this.findConnection(
                    results.intersection,
                    startNodeMap,
                    endNodeMap,
                    startNode,
                    endNode
                );
                if (connection) {
                    var path = Util.biBacktrace(connection.viaStart, connection.viaEnd);
                    // Print the path for debugging
                    for (i = 0; i < path.length; i++) {
                        console.log("Path node:", path[i][0], path[i][1]);
                    }
                    return path;
                }
                console.warn(
                    "Connection failed to find a valid via node pair, continuing search"
                );
            }
        }
    } // end while not open list empty

    // fail to find the path
    return [];
}

BubbleStarFinder.prototype.findPath = function(
    startX,
    startY,
    endX,
    endY,
    grid
) {
    if (this.connect) {
        return this.findPathConnect.call(this, startX, startY, endX, endY, grid);
    }
    return this.findPathOneDirection.call(this, startX, startY, endX, endY, grid);
};

module.exports = BubbleStarFinder;
