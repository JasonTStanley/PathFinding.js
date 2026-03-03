var Heap = require('heap')
var Util = require('../core/Util')
var Heuristic = require('../core/Heuristic')
var DiagonalMovement = require('../core/DiagonalMovement')

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
function BubbleStarFinder (opt) {
  opt = opt || {}
  this.allowDiagonal = opt.allowDiagonal
  this.dontCrossCorners = opt.dontCrossCorners
  this.heuristic = opt.heuristic || Heuristic.euclidean
  this.weight = opt.weight || 1
  this.diagonalMovement = opt.diagonalMovement
  this.debug = !!opt.debug
  this.onStep = typeof opt.onStep === 'function' ? opt.onStep : null
  this.debuggerBreak = !!opt.debuggerBreak
  this.maxIterations = opt.maxIterations || 50000

  if (!this.diagonalMovement) {
    if (!this.allowDiagonal) {
      this.diagonalMovement = DiagonalMovement.Never
    } else {
      if (this.dontCrossCorners) {
        this.diagonalMovement = DiagonalMovement.OnlyWhenNoObstacles
      } else {
        this.diagonalMovement = DiagonalMovement.IfAtMostOneObstacle
      }
    }
  }

  // When diagonal movement is allowed the manhattan heuristic is not
  //admissible. It should be octile instead
  if (this.diagonalMovement === DiagonalMovement.Never) {
    this.heuristic = opt.heuristic || Heuristic.manhattan
  } else {
    this.heuristic = opt.heuristic || Heuristic.octile
  }
}

BubbleStarFinder.prototype._buildOccupiedCellList = function (grid) {
  var occupied = []
  var x
  var y

  for (x = 0; x < grid.width; ++x) {
    for (y = 0; y < grid.height; ++y) {
      if (!grid.isWalkableAt(x, y)) {
        occupied.push([x, y])
      }
    }
  }

  return occupied
}

BubbleStarFinder.prototype.signedDistanceAt = function (
  x,
  y,
  grid,
  occupiedCells
) {
  var nearest
  var i
  var dx
  var dy
  var dist

  if (!grid.isInside(x, y)) {
    return 0
  }

  occupiedCells = occupiedCells || this._buildOccupiedCellList(grid)

  if (!occupiedCells.length) {
    nearest = Math.max(grid.width, grid.height)
  } else {
    nearest = Infinity
    for (i = 0; i < occupiedCells.length; ++i) {
      dx = x - occupiedCells[i][0]
      dy = y - occupiedCells[i][1]
      dist = Math.sqrt(dx * dx + dy * dy)
      if (dist < nearest) {
        nearest = dist
      }
    }
  }

  return nearest
}

function key (node) {
  if (typeof node === 'object') {
    return node.x + ',' + node.y
  }
  return node + ',' + arguments[1]
}

function diskBoundaryOffsets (radius, consider_diagonal) {
  var out = []
  if (radius <= 0) return out

  var R2 = radius * radius

  // 8-neighborhood
  var N
  if (consider_diagonal) {
    N = [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
      [1, 1],
      [-1, 1],
      [1, -1],
      [-1, -1]
    ]
  } else {
    N = [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1]
    ]
  }

  // range(floor(-r), ceil(r+1)) with hi exclusive
  var lo = Math.floor(-radius)
  var hi = Math.ceil(radius + 1.0)

  for (var x = lo; x < hi; x++) {
    for (var y = lo; y < hi; y++) {
      var p2 = x * x + y * y

      // inside test: strictly inside
      if (p2 >= R2) continue

      // boundary test: any 8-neighbor outside (or on) the circle
      var isBoundary = false
      for (var i = 0; i < N.length; i++) {
        var nx = x + N[i][0]
        var ny = y + N[i][1]
        var q2 = nx * nx + ny * ny

        if (q2 >= R2) {
          isBoundary = true
          break
        }
      }

      if (!isBoundary) continue
      out.push([x, y])
    }
  }

  return out
}

function computeDistanceMatrix2D (V, Q) {
  var M = V.length
  var N = Q.length

  var D = new Array(M)
  for (var i = 0; i < M; i++) {
    D[i] = new Array(N)
    var vx = V[i][0]
    var vy = V[i][1]

    for (var j = 0; j < N; j++) {
      var dx = vx - Q[j][0]
      var dy = vy - Q[j][1]
      D[i][j] = Math.sqrt(dx * dx + dy * dy)
    }
  }

  return D
}

function openWithinRadius (openMap, qx, qy, r) {
  var r2 = r * r
  var r_int = Math.ceil(r)
  var out = []
  for (var dx = -r_int; dx <= r_int; dx++) {
    for (var dy = -r_int; dy <= r_int; dy++) {
      if (dx * dx + dy * dy >= r2) continue // keep circle
      var n = openMap.get(key(qx + dx, qy + dy))
      if (n) out.push(n)
    }
  }
  return out
}

function Bubble (x, y, radius) {
  this.x = x
  this.y = y
  this.radius = radius
}

BubbleStarFinder.prototype.findPath = function (
  startX,
  startY,
  endX,
  endY,
  grid
) {
  var openList = new Heap(function (nodeA, nodeB) {
      return nodeA.f - nodeB.f
    }),
    openMap = new Map(),
    bubbles = [],
    startNode = grid.getNodeAt(startX, startY),
    endNode = grid.getNodeAt(endX, endY),
    heuristic = this.heuristic,
    diagonalMovement = this.diagonalMovement,
    weight = this.weight,
    abs = Math.abs,
    SQRT2 = Math.SQRT2,
    node,
    i,
    l,
    x,
    y

  function estimateHeuristic (x, y) {
    return weight * heuristic(abs(x - endX), abs(y - endY))
  }

  var occupiedCells = this._buildOccupiedCellList(grid)

  // set the `g` and `f` value of the start node to be 0
  startNode.g = 0
  startNode.f = 0

  // push the start node into the open list
  openList.push(startNode)
  startNode.opened = true
  openMap.set(key(startNode), startNode)

  /**
   * Compute neighbors for Bubble* expansion from a node.
   *
   * @param {Object} node          Current node {x,y,cost,parent}
   * @param {number} radius        Radius in world units OR grid units depending on resolution
   *
   * @returns {Array<Object>} neighbors nodes (new objects) with {x,y,cost,parent}
   */
  function expandAndUpdateBoundary (node, radius, bubble_idx) {
    var neighbors = []

    // Convert radius to integer cell radius, like C++: floor(radius / resolution_)
    var r = Math.floor(radius)
    if (r <= 0) return neighbors

    // "sphereEdge" in 2D => your disk boundary offsets for integer radius r
    var edge = diskBoundaryOffsets(r, diagonalMovement) // returns Array<[dx,dy]>

    // Base case: no parent
    if (!node.parent) {
      for (var i = 0; i < edge.length; i++) {
        var dx = edge[i][0],
          dy = edge[i][1]
        var nx0 = node.x + dx
        var ny0 = node.y + dy
        if (!grid.isInside(nx0, ny0) || !grid.isWalkableAt(nx0, ny0)) {
          continue
        }
        var stepCost = Math.hypot(dx, dy)
        var neighbor = grid.getNodeAt(nx0, ny0)
        neighbor.g = node.g + stepCost
        neighbor.h = neighbor.h || estimateHeuristic(nx0, ny0)
        neighbor.f = neighbor.g + neighbor.h
        neighbor.parent = node
        neighbor.bubble_idx = bubble_idx
        neighbors.push(neighbor)
      }
      return neighbors
    }

    // Candidate "via" nodes near current node — use OPEN set within r
    var viaNodes = openWithinRadius(openMap, node.x, node.y, r - 1)
    var K = viaNodes.length

    if (viaNodes.length > 0) {
      // Precompute bubbles referenced by via nodes (and guard bubble_idx)
      var viaBubbles = []
      for (var i = 0; i < viaNodes.length; i++) {
        var via = viaNodes[i]
        // Mark via as closed to prevent reuse in this expansion (lazy deletion)
        via.closed = true
        // Remove from openMap to show removal from the open set, we can do this immediately
        openMap.delete(key(via))

        var idx = via.bubble_idx
        if (idx != null && bubbles[idx]) viaBubbles.push(bubbles[idx])
      }

      // Filter edge once
      var filteredEdge = []
      for (i = 0; i < edge.length; i++) {
        dx = edge[i][0]
        dy = edge[i][1]
        var nx = node.x + dx
        var ny = node.y + dy
        var insideViaBubble = false

        for (var j = 0; j < viaBubbles.length; j++) {
          var b = viaBubbles[j]
          var bx = nx - b.x
          var by = ny - b.y
          b_rad = Math.floor(b.radius)
          if (bx * bx + by * by < b_rad * b_rad) {
            insideViaBubble = true
            break
          }
        }

        if (!insideViaBubble) {
          filteredEdge.push(edge[i])
        }
      }
      edge = filteredEdge
    }

    var N = edge.length

    if (N === 0) {
      console.warn(
        'Bubble* warning: all edge neighbors were inside via bubbles, skipping this bubble',
        radius,
        node.x,
        node.y
      )
      return neighbors
    }

    // If none found (can happen if your openMap excludes some needed nodes),
    // you need a fallback. Closest match to intent: fall back to using node as via.
    // This keeps the algorithm progressing.
    if (K === 0) {
      console.warn(
        'Bubble* fallback: no open nodes found within radius, using current node as via'
      )
      for (i = 0; i < N; i++) {
        ;(dx = edge[i][0]), (dy = edge[i][1])
        nx = node.x + dx
        ny = node.y + dy
        if (!grid.isInside(nx, ny) || !grid.isWalkableAt(nx, ny)) {
          continue
        }
        stepCost = Math.hypot(dx, dy)
        var neighbor = grid.getNodeAt(nx, ny)
        if (!neighbor.opened || node.g + stepCost < neighbor.g) {
          neighbor.g = node.g + stepCost
          neighbor.h = estimateHeuristic(nx, ny)
          neighbor.f = neighbor.g + neighbor.h
          neighbor.parent = node
          neighbor.bubble_idx = bubble_idx
          neighbors.push(neighbor)
        }
      }
      return neighbors
    }

    // For each edge step, choose best via: min_j (via.cost + dist(via.pos, next))
    for (i = 0; i < N; i++) {
      ;(dx = edge[i][0]), (dy = edge[i][1])
      nx = node.x + dx
      ny = node.y + dy
      if (!grid.isInside(nx, ny) || !grid.isWalkableAt(nx, ny)) {
        continue
      }

      var bestVia = viaNodes[0]
      var bestCost = Infinity

      for (j = 0; j < K; j++) {
        var v = viaNodes[j]
        var dist = Math.hypot(v.x - nx, v.y - ny)
        var total = v.g + dist

        if (total < bestCost) {
          bestCost = total
          bestVia = v
        }
      }

      var neighbor = grid.getNodeAt(nx, ny)
      // check if we have a better cost and update the neighbor
      if (!neighbor.opened || bestCost < neighbor.g) {
        neighbor.g = bestCost
        neighbor.h = estimateHeuristic(nx, ny)
        neighbor.f = neighbor.g + neighbor.h
        neighbor.parent = bestVia
        neighbor.bubble_idx = bubble_idx
        neighbors.push(neighbor)
      }
    }

    return neighbors
  }

  function bubbleContains (bubble, node) {
    var dx = node.x - bubble.x
    var dy = node.y - bubble.y
    var distance_sq = dx * dx + dy * dy
    return distance_sq < bubble.radius * bubble.radius
  }

  // while the open list is not empty
  while (!openList.empty()) {
    // pop the position of node which has the minimum `f` value.
    node = openList.pop()
    if (node === endNode || endNode.closed) {
      return Util.backtrace(endNode)
    }

    if (node.closed) {
      // lazy deletion: skip nodes already closed
      continue
    }
    node.closed = true

    var radius = this.signedDistanceAt(node.x, node.y, grid, occupiedCells)

    var bubble = new Bubble(node.x, node.y, radius)
    bubbles.push(bubble)

    // if reached the end position, construct the path and return it
    if (bubbleContains(bubble, endNode)) {
      // calculate the path to the end node,
      var viaNodes = openWithinRadius(openMap, node.x, node.y, radius)
      viaNodes.push(node) // also consider the current node as a via candidate
      var bestCost = Infinity
      for (i = 0; i < viaNodes.length; i++) {
        var via = viaNodes[i]
        dist = Math.hypot(via.x - endNode.x, via.y - endNode.y)
        total = via.g + dist
        if (total < bestCost) {
          bestCost = total
          endNode.parent = via
          endNode.g = total
          endNode.h = 0
          endNode.f = total
          endNode.bubble_idx = bubbles.length - 1
        }
      }
      endNode.opened = true
      openList.push(endNode)
    }

    // get neigbours of the current node
    neighbors = expandAndUpdateBoundary(node, radius, bubbles.length - 1)
    for (i = 0, l = neighbors.length; i < l; ++i) {
      neighbor = neighbors[i]

      if (neighbor.closed) {
        continue
      }

      x = neighbor.x
      y = neighbor.y
      if (!neighbor.opened) {
        openList.push(neighbor)
        neighbor.opened = true
        openMap.set(key(neighbor), neighbor)
      } else {
        // the neighbor can be reached with smaller cost.
        // Since its f value has been updated, we have to
        // update its position in the open list
        openList.updateItem(neighbor)
      }
    } // end for each neighbor
  } // end while not open list empty

  // fail to find the path
  return []
}

module.exports = BubbleStarFinder
