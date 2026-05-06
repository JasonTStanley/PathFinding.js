var BubbleStarFinder = require('./BubbleStarFinder');

/**
 * Bi-directional Bubble* path-finder.
 * @constructor
 * @extends BubbleStarFinder
 * @param {Object} opt
 */
function BiBubbleStarFinder(opt) {
    opt = opt || {};
    opt.connect = true;
    BubbleStarFinder.call(this, opt);
}

BiBubbleStarFinder.prototype = new BubbleStarFinder();
BiBubbleStarFinder.prototype.constructor = BiBubbleStarFinder;

module.exports = BiBubbleStarFinder;
