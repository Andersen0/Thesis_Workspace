// The size of the SVG container
const width = 800;
const height = 800;

// The radius of the larger circle
const radius = Math.min(width, height) / 2 - 40;

// Adjust the scale to start from 0
const scale = d3.scaleLinear()
  .domain([0, 30])
  .range([0, 2 * Math.PI]);

// The SVG container
const svg = d3.select('body').append('svg')
  .attr('width', width)
  .attr('height', height)
  .append('g')
  .attr('transform', 'translate(' + width / 2 + ',' + height / 2 + ')');

// Array to store the messages
let messages = [];

// Define the arrow marker
svg.append('defs').append('marker')
  .attr('id', 'arrow')
  .attr('viewBox', '0 -5 10 10')
  .attr('refX', 5)
  .attr('refY', 0)
  .attr('markerWidth', 5)
  .attr('markerHeight', 5)
  .attr('orient', 'auto')
  .append('path')
  .attr('d', 'M0,-5L10,0L0,5')
  .attr('class', 'arrowHead');

// Append a line to serve as the indicator and use the marker for the arrow
let indicator = svg.append('line')
  .attr('stroke', 'black')
  .attr('stroke-width', 8)
  .attr('marker-end', 'url(#arrow)')
  .attr('x1', 0)
  .attr('y1', 0)
  .attr('x2', 0)
  .attr('y2', -0.8*radius);

// Function to update the visualization
function update() {
  // Bind the messages array to the circles in the SVG container, using the index as the key
  let circles = svg.selectAll('.messageCircle').data(messages, function(d, i) { return i; });

  // For any new messages, append a new circle
  let enter = circles.enter().append('circle')
    .attr('class', 'messageCircle');

  // Merge the new elements with the existing ones
  circles = enter.merge(circles);

  circles
    .attr('r', function(d, i) {
      return i === 0 ? 20 : 15;  // increase the radius for the top circle
    })
    .attr('fill', function(d) {
      // The color of the circle depends on the message
      switch (d) {
        case 'idle': return 'blue';
        case 'moving': return 'green';
        case 'stopped': return 'red';
      }
    })
    .attr('transform', function(d, i) {
      // Position the circle around the edge of the larger circle
      let angle = scale(i);
      return 'rotate(' + (angle * 180 / Math.PI - 90) + ')translate(' + radius + ')';
    });

  // For any old messages that have been removed, remove the circle
  circles.exit().remove();

  // Remove the existing border circle
  svg.selectAll('.borderCircle').remove();

  // Draw a border circle around the top circle
  svg.append('circle')
    .attr('class', 'borderCircle')
    .attr('r', 23) // radius should be greater than that of message circles
    .attr('fill', 'none')
    .attr('stroke', 'black')
    .attr('stroke-width', 6) // increase the stroke width
    .attr('transform', function() {
      // Position the circle at the top
      let angle = scale(0);
      return 'rotate(' + (angle * 180 / Math.PI - 90) + ')translate(' + radius + ')';
    });
}

// Store the most recent status message
let lastStatusMessage = null;

function updateStatusMessages() {
  fetch('/status')
    .then(response => response.json())
    .then(fetchedMessages => { // renamed to avoid shadowing
      const ul = document.getElementById('recent_status_messages');
      ul.innerHTML = '';

      // We'll assume the most recent message is at the end of the array
      const mostRecentMessage = fetchedMessages[fetchedMessages.length - 1];

      // Only update the status circle color if a new message has been received
      if (mostRecentMessage == 'status: idle') {
        messages.unshift('idle');  // Add to the beginning of the array
        document.getElementById('status-circle').style.backgroundColor = 'blue';
        lastStatusMessage = mostRecentMessage;
      } else if (mostRecentMessage == 'status: moving') {
        messages.unshift('moving'); // Add to the beginning of the array
        document.getElementById('status-circle').style.backgroundColor = 'green';
      } else {
        messages.unshift('stopped'); // Add to the beginning of the array
        document.getElementById('status-circle').style.backgroundColor = 'red';
      }

      // If there are already 60 messages, remove the oldest one
      if (messages.length > 29) {
        messages.pop();
      }

      // Call the update function to redraw the circles
      update();

      for (const message of fetchedMessages) {
        const newMessage = document.createElement('li');
        newMessage.textContent = message;
        ul.appendChild(newMessage);
      }
    });
}

// Update status messages every 1 second
setInterval(updateStatusMessages, 1000);