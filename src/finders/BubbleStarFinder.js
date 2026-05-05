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

function canMoveDiagonally(diagonalMovement) {
    return diagonalMovement !== DiagonalMovement.Never;
}

function key(node) {
    if (typeof node === "object") {
        return node.x + "," + node.y;
    }
    return node + "," + arguments[1];
}

// TODO: Inclusive Check
function checkDistance(dx, dy, radius) {
    var distance_sq = dx * dx + dy * dy;
    return distance_sq <= radius * radius;
}

function generateSuccessorCandidates(radius, consider_diagonal) {
    var out = [];
    if (radius <= 0) return out;

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
            // inside test: if outside the radius, skip this cell
            if (!checkDistance(x, y, radius)) continue;
            // skip the center cell
            if (x === 0 && y === 0) continue;

            // boundary test: any 8-neighbor outside the circle
            var isBoundary = false;
            for (var i = 0; i < N.length; i++) {
                var nx = x + N[i][0];
                var ny = y + N[i][1];
                // if any neighbor is outside the radius, then this is a boundary cell
                if (!checkDistance(nx, ny, radius)) {
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

function findVias(nodeMap, qx, qy, r) {
    var out = [];
    for (var dx = -r; dx <= r; dx++) {
        for (var dy = -r; dy <= r; dy++) {
            if (!checkDistance(dx, dy, r)) continue;
            var n = nodeMap.get(key(qx + dx, qy + dy));
            if (n) out.push(n);
        }
    }
    return out;
}

function cullSuccessors(edge, node, viaBubbles) {
    var filteredEdge = [];
    for (var i = 0; i < edge.length; i++) {
        var dx = edge[i][0];
        var dy = edge[i][1];
        var nx = node.x + dx;
        var ny = node.y + dy;
        var insideViaBubble = false;

        for (var j = 0; j < viaBubbles.length; j++) {
            var b = viaBubbles[j];
            var bx = nx - b.x;
            var by = ny - b.y;
            if (checkDistance(bx, by, b.radius)) {
                insideViaBubble = true;
                break;
            }
        }

        if (!insideViaBubble) {
            filteredEdge.push(edge[i]);
        }
    }
    return filteredEdge;
}

function Bubble(x, y, radius) {
    this.x = x;
    this.y = y;
    this.radius = radius;
}

function bubbleContains(bubble, node) {
    var dx = node.x - bubble.x;
    var dy = node.y - bubble.y;
    return checkDistance(dx, dy, bubble.radius);
}

function findOverlap(node, successors, bubbles, bubble_idx) {
    // Implementation for checking overlap between a node and its successors
    // Only for Bi-directional: track which boundary (start vs end) sees this neighbor, for meeting-in-the-middle detection
    var bubble_overlap = null;
    var i, neighbor;
    for (i = 0; i < successors.length; ++i) {
        neighbor = successors[i].neighbor;
        if (neighbor.by && neighbor.by != node.by) {
            // if this neighbor is already opened by the other search direction, we have a meeting in the middle! Track the bubble that caused this for connection finding.
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
            bubble_overlap = bubbles[bubble_idx];
        }
    }
    // Note: reassigning the local parameter won't affect the caller's reference.
    // If callers need filtered successors they should use the returned bubble and the existing successors array.
    return bubble_overlap;
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

BubbleStarFinder.prototype.estimateHeuristic = function(goalX, goalY, x, y) {
    return this.weight * this.heuristic(Math.abs(x - goalX), Math.abs(y - goalY));
};

/**
 * Compute successors for Bubble* expansion from a node.
 *
 * @param {number} goalX        Goal x (for heuristic)
 * @param {number} goalY        Goal y (for heuristic)
 * @param {Object} node          Current node {x,y,cost,parent}
 * @param {number} radius        Radius in world units OR grid units depending on resolution
 * @param {number} bubble_idx    Index of the bubble being expanded (for book-keeping)
 *
 * @returns {Array<Object>} successors nodes (new objects) with {x,y,cost,parent}
 */
BubbleStarFinder.prototype.calculateSuccessors = function(
    grid,
    nodeMap,
    bubbles,
    node,
    radius
) {
    var successors = [];
    var considerDiagonal = canMoveDiagonally(this.diagonalMovement);
    var i;
    var j;
    var dx;
    var dy;
    var nx;
    var ny;
    var stepCost;

    if (radius < 0.5) return successors;

    // "sphereEdge" in 2D -> your disk boundary offsets for integer radius r
    var edge = generateSuccessorCandidates(radius, considerDiagonal); // returns Array<[dx,dy]>

    // Base case: no parent
    if (!node.parent) {
        for (i = 0; i < edge.length; i++) {
            dx = edge[i][0];
            dy = edge[i][1];
            var nx = node.x + dx;
            var ny = node.y + dy;

            if (!grid.isWalkableAt(nx, ny)) continue;

            stepCost = Math.hypot(dx, dy);
            var neighbor = grid.getNodeAt(nx, ny);

            // TODO: neighbor.h = neighbor.h || estimateHeuristic(nx, ny);
            successors.push({ neighbor: neighbor, bestVia: node, bestCost: node.g + stepCost });
        }
        nodeMap.delete(key(node));
        return successors;
    }

    // Candidate "via" nodes near current node — use OPEN set within r
    var viaNodes = findVias(nodeMap, node.x, node.y, radius);
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

        // Filter edge once based on via bubbles, to avoid redundant checks for each via candidate.
        edge = cullSuccessors(edge, node, viaBubbles);
    }

    // Process Filtered edge:
    if (edge.length === 0) {
        console.warn(
            "Bubble* warning: all edge successors were inside via bubbles, skipping this bubble",
            radius,
            node.x,
            node.y
        );
        return successors;
    }


    // For each edge step, choose best via: min_j (via.cost + dist(via.pos, next))
    for (i = 0; i < edge.length; i++) {
        dx = edge[i][0];
        dy = edge[i][1];
        nx = node.x + dx;
        ny = node.y + dy;

        if (!grid.isWalkableAt(nx, ny)) continue;

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

        var successor = { neighbor: neighbor, bestVia: bestVia, bestCost: bestCost };
        successors.push(successor);
    }

    return successors;
};

BubbleStarFinder.prototype.resolveOverlap = function(
    bubbleOverlap,
    forwardNodeMap,
    backwardNodeMap,
    startNode,
    endNode
) {
    var bubble = bubbleOverlap;

    if (!bubble) {
        return null;
    }

    // Candidate "via" nodes near current node — use OPEN set within r
    var viaNodesStart = findVias(
        forwardNodeMap,
        bubble.x,
        bubble.y,
        bubble.radius
    );
    var viaNodesEnd = findVias(
        backwardNodeMap,
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

    // TODO: A Fallback check, may be unnecessary if the bubble overlap check is strict enough
    if (!bestViaStart || !bestViaEnd) {
        console.warn(
            "Connection failed to find a valid via node pair, this should be rare. Returning null to continue search."
        );
        return null;
    }

    return { viaStart: bestViaStart, viaEnd: bestViaEnd };
};

BubbleStarFinder.prototype.handleEndCondition = function(
    node,
    successors,
    bubbles,
    bubble_idx,
    endNode,
    forwardNodeMap,
    startNode,
    backwardNodeMap
) {
    // Implementation for handling end condition
    startNode = startNode || null;
    backwardNodeMap = backwardNodeMap || null;
    
    // Bi-directional case: check for meeting in the middle
    if (node.by) {
        // Check for meeting in the middle by seeing if any successor is already opened by the other search direction
        var last_bubble = findOverlap(node, successors, bubbles, bubble_idx);
        if (last_bubble) {
            console.log("Meeting in the middle detected!");
            var overlap_solution = this.resolveOverlap(last_bubble, forwardNodeMap, backwardNodeMap, startNode, endNode);
            if (overlap_solution) {
                var path = Util.biBacktrace(overlap_solution.viaStart, overlap_solution.viaEnd);
                // Print the path for debugging
                for (var i = 0; i < path.length; i++) {
                    console.log("Path node:", path[i][0], path[i][1]);
                }
                return path;
            }
            console.warn(
                "Connection failed to find a valid via node pair, continuing search"
            );
        }
    } else {
        // if reached the end position, construct the path and return it
        var last_bubble = bubbles[bubble_idx];
        if (bubbleContains(last_bubble, endNode)) {
            console.log("End node is within bubble, connecting directly to end node");
            // calculate the path to the end node,
            var viaNodes = findVias(forwardNodeMap, last_bubble.x, last_bubble.y, last_bubble.radius);
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

            var path = Util.backtrace(endNode);
            // Print the path for debugging
            for (i = 0; i < path.length; i++) {
                console.log("Path node:", path[i][0], path[i][1]);
            }
            return path;
        }
    }
    return null; // if not bi-directional or no overlap, return null to continue normal processing
};

BubbleStarFinder.prototype.findPathOneDirection = function(
    startX,
    startY,
    endX,
    endY,
    grid
) {
    var cmp = function(a, b) {
        return a.f - b.f;
    };
    var openList = new Heap(cmp);
    var nodeMap = new Map();
    var bubbles = [];
    var startNode = grid.getNodeAt(startX, startY);
    var endNode = grid.getNodeAt(endX, endY);

    var node;

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

        // lazy deletion: skip nodes already closed
        if (!node.closed) {
            node.closed = true;

            var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
            radius = Math.floor(radius);
            console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
            var bubble = new Bubble(node.x, node.y, radius);
            bubbles.push(bubble);

            // get neigbours of the current node
            var successors = this.calculateSuccessors(grid, nodeMap, bubbles, node, radius);

            var path = this.handleEndCondition(node, successors, bubbles, bubbles.length - 1, endNode, nodeMap);
            if (path) {
                return path;
            }

            for (var i = 0; i < successors.length; ++i) {
                var successor = successors[i];
                var neighbor = successor.neighbor;
                var bestVia = successor.bestVia;
                var bestCost = successor.bestCost;

                if (!neighbor.opened || bestCost < neighbor.g) {
                    // check if we have a better cost and update the neighbor
                    neighbor.g = bestCost;
                    neighbor.h = this.estimateHeuristic(endX, endY, neighbor.x, neighbor.y);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = bestVia;
                    neighbor.bubble_idx = bubbles.length - 1;
                } else {
                    continue;
                }

                if (neighbor.closed) {
                    continue;
                }

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
        }
    } // end while not open list empty

    // fail to find the path
    return [];
};

BubbleStarFinder.prototype.findPathConnect = function(startX, startY, endX, endY, grid) {
    var cmp = function(a, b) {
        return a.f - b.f;
    };
    var forwardOpenList = new Heap(cmp);
    var backwardOpenList = new Heap(cmp);
    var forwardNodeMap = new Map();
    var backwardNodeMap = new Map();
    var forwardBubbles = [];
    var backwardBubbles = [];
    var startNode = grid.getNodeAt(startX, startY);
    var endNode = grid.getNodeAt(endX, endY);
    var BY_START = 1,
        BY_END = 2;

    var node;

    var occupiedCells = this._buildOccupiedCellList(grid);

    // set the `g` and `f` value of the start & end nodes to be 0
    startNode.g = 0;
    startNode.f = 0;
    endNode.g = 0;
    endNode.f = 0;

    // push the start node into the start open list
    forwardOpenList.push(startNode);
    startNode.opened = true;
    startNode.by = BY_START;
    forwardNodeMap.set(key(startNode), startNode);

    // push the end node into the end open list
    backwardOpenList.push(endNode);
    endNode.opened = true;
    endNode.by = BY_END;
    backwardNodeMap.set(key(endNode), endNode);

    // while the open lists are not empty
    while (!forwardOpenList.empty() && !backwardOpenList.empty()) {
        // FORWARD EXPANSION
        // pop the position of node which has the minimum `f` value.
        node = forwardOpenList.pop();

        // lazy deletion: skip nodes already closed
        if (!node.closed) {
            node.closed = true;

            var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
            radius = Math.floor(radius);
            console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
            var bubble = new Bubble(node.x, node.y, radius);
            forwardBubbles.push(bubble);

            // get neigbours of the current node
            var successors = this.calculateSuccessors(grid, forwardNodeMap, forwardBubbles, node, radius);

            // Check for end condition: if reached the end position
            var path = this.handleEndCondition(node, successors, forwardBubbles, forwardBubbles.length - 1, endNode, forwardNodeMap, startNode, backwardNodeMap);
            if (path) {
                return path;
            }

            // Update open list and node map for each neighbor
            for (var i = 0; i < successors.length; ++i) {
                var successor = successors[i];
                var neighbor = successor.neighbor;
                var bestVia = successor.bestVia;
                var bestCost = successor.bestCost;

                if (!neighbor.opened || bestCost < neighbor.g) {
                    // check if we have a better cost and update the neighbor
                    neighbor.g = bestCost;
                    neighbor.h = this.estimateHeuristic(endX, endY, neighbor.x, neighbor.y);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = bestVia;
                    neighbor.bubble_idx = forwardBubbles.length - 1;
                } else {
                    continue;
                }

                if (neighbor.closed) {
                    continue;
                }

                if (!neighbor.opened) {
                    forwardOpenList.push(neighbor);
                    neighbor.opened = true;
                    neighbor.by = BY_START;
                    forwardNodeMap.set(key(neighbor), neighbor);
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    forwardOpenList.updateItem(neighbor);
                }
            } // end for each neighbor
        }

        // BACKWARD EXPANSION
        // pop the position of node which has the minimum `f` value.
        node = backwardOpenList.pop();

        // lazy deletion: skip nodes already closed
        if (!node.closed) {
            node.closed = true;

            var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells);
            radius = Math.floor(radius);
            console.log("Expanding bubble at", node.x, node.y, "with radius", radius);
            var bubble = new Bubble(node.x, node.y, radius);
            backwardBubbles.push(bubble);

            // get neigbours of the current node
            var successors = this.calculateSuccessors(grid, backwardNodeMap, backwardBubbles, node, radius);

            // Check for end condition: if reached the end position
            var path = this.handleEndCondition(node, successors, backwardBubbles, backwardBubbles.length - 1, endNode, forwardNodeMap, startNode, backwardNodeMap);
            if (path) {
                return path;
            }

            // Update open list and node map for each neighbor
            for (var i = 0; i < successors.length; ++i) {
                var successor = successors[i];
                var neighbor = successor.neighbor;
                var bestVia = successor.bestVia;
                var bestCost = successor.bestCost;

                if (!neighbor.opened || bestCost < neighbor.g) {
                    // check if we have a better cost and update the neighbor
                    neighbor.g = bestCost;
                    neighbor.h = this.estimateHeuristic(startX, startY, neighbor.x, neighbor.y);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = bestVia;
                    neighbor.bubble_idx = backwardBubbles.length - 1;
                } else {
                    continue;
                }

                if (neighbor.closed) {
                    continue;
                }

                if (!neighbor.opened) {
                    backwardOpenList.push(neighbor);
                    neighbor.opened = true;
                    neighbor.by = BY_END;
                    backwardNodeMap.set(key(neighbor), neighbor);
                } else {
                    // the neighbor can be reached with smaller cost.
                    // Since its f value has been updated, we have to
                    // update its position in the open list
                    backwardOpenList.updateItem(neighbor);
                }
            } // end for each neighbor
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
