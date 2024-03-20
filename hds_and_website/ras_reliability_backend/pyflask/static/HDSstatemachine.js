var HDSContainer = d3.select('#HDSvisualisation');

// Constant ellipse parameters for radii
const ellipseRadiusX = 80;
const ellipseRadiusY = 45;

// Get the center coordinates of ellipse1 and ellipse2 based on the center of the HDSContainer svg
const ellipse1CenterX = HDSContainer.attr("width") / 2 - 100;
const ellipse1CenterY = HDSContainer.attr("height") / 2 + 150;
const ellipse2CenterX = HDSContainer.attr("width") / 2 + 100;
const ellipse2CenterY = HDSContainer.attr("height") / 2 + 150;
const ellipse3CenterX = HDSContainer.attr("width") / 2 - 200;

// Calculate the difference in coordinates between the ellipse centers
const xDiff = ellipse2CenterX - ellipse1CenterX;
const yDiff = ellipse2CenterY - ellipse1CenterY;

// Calculate control points for the curve based on ellipse positions and radii
const controlPoint1X = ellipse1CenterX + ellipseRadiusX;
const controlPoint1Y = ellipse1CenterY - yDiff / 2;

const controlPoint2X = ellipse2CenterX - ellipseRadiusX;
const controlPoint2Y = ellipse2CenterY - yDiff / 2;

const nodeTextSize = 16;
const labelTextSize = 16;

// Function to draw a rectangle and return it
function drawRectangle(container, centerX, centerY, width, height, fill, text, rectID, rectClass) {
  const rectangle = container
    .append("rect")
    .attr("x", centerX - width / 2)
    .attr("y", centerY - height / 2)
    .attr("width", width)
    .attr("height", height)
    .attr("fill", fill)
    .attr("stroke", "black")
    .attr("stroke-width", 1)
    .attr("id", rectID)
    .attr("class", rectClass)
    .attr("data-default-fill", fill);

  container
    .append("text")
    .text(text)
    .attr("x", centerX)
    .attr("y", centerY)
    .attr("text-anchor", "middle")
    .attr("dominant-baseline", "central")
    .attr("font-size", nodeTextSize);
}



function drawStraightLink(container, x1, y1, x2, y2, labelText) {
  const link = container
    .append("line")
    .attr("x1", x1)
    .attr("y1", y1)
    .attr("x2", x2 - ellipseRadiusX * 2) // Subtract an ellipseRadiusX here to adjust the position
    .attr("y2", y2)
    .attr("stroke", "black")
    .attr("stroke-width", 1);

  const label = container
    .append("text")
    .text(labelText)
    .attr("font-size", labelTextSize)
    .attr("text-anchor", "middle")
    .attr("dominant-baseline", "central")
    .attr("dy", "-0.5em");

  // Update the label's position and orientation based on the link's slope
  function updateLabelPosition() {
    const labelX = (x1 + x2 - ellipseRadiusX * 2) / 2; // Subtract an ellipseRadiusX here as well
    const labelY = (y1 + y2) / 2;
    const labelAngle = Math.atan2(y2 - y1, x2 - x1 + ellipseRadiusX * 2) * (180 / Math.PI);
    label.attr("transform", `translate(${labelX}, ${labelY}) rotate(${labelAngle})`);
  }

  // Call the updateLabelPosition function initially and whenever the link positions change
  updateLabelPosition();
}


function drawCurvedLink(container, centerX1, centerY1, centerX2, centerY2, radiusX, radiusY, labelText) {
  container
    .append("path")
    .attr("d", `M ${centerX1} ${centerY1 - radiusY} A ${radiusX} ${radiusY} 0 0 1 ${centerX2} ${centerY2 - radiusY}`)
    .attr("fill", "none")
    .attr("stroke", "black")
    .attr("stroke-width", 1);

  const labelYOffset = 120; // Adjust this value to control the distance of the label above the curved link
  container
    .append("text")
    .text(labelText)
    .attr("x", (centerX1 + centerX2) / 2)
    .attr("y", centerY1 - radiusY - labelYOffset)
    .attr("text-anchor", "middle")
    .attr("dominant-baseline", "bottom")
    .attr("font-size", labelTextSize);
}


let ellipse1, ellipse2;

// First rectangle
const rect1HDS = drawRectangle(HDSContainer, ellipse1CenterX, ellipse1CenterY, ellipseRadiusX * 2, ellipseRadiusY * 2, "lightblue", "UV treatment", "rect1HDS", "status-rectangle4");

// Add another rectangle at the same height as the first one, but to the right of it
const rect2HDS = drawRectangle(HDSContainer, ellipse2CenterX, ellipse2CenterY, ellipseRadiusX * 2, ellipseRadiusY * 2, "lightblue", "UV stop", "rect2HDS", "status-rectangle5");

// Add third rectangle in the middle of the first two, but higher up
const rect3HDS = drawRectangle(HDSContainer, ellipse3CenterX, ellipse1CenterY - 300, ellipseRadiusX * 2, ellipseRadiusY * 2, "lightblue", "Restart", "rect3HDS", "status-rectangle6");

// Add link between the two rectangles
const linkHDS1 = drawStraightLink(HDSContainer, ellipse1CenterX + ellipseRadiusX, ellipse1CenterY, ellipse2CenterX + ellipseRadiusX, ellipse2CenterY, "Restart");

// Add link that curves smoothly from the first rectangle to the third rectangle
const linkHDS2 = drawCurvedLink(HDSContainer, ellipse1CenterX - 40 + ellipseRadiusX, ellipse1CenterY - 30, ellipse3CenterX - ellipseRadiusX, ellipse1CenterY - 290, ellipseRadiusX / 7, ellipseRadiusY / 4, "event 1");

function updateStatusMessages() {
  fetch('/status')
    .then(response => response.json())
    .then(messages => {
      const ul = document.getElementById('recent_status_messages');
      ul.innerHTML = '';

      // We'll assume the most recent message is at the end of the array
      const mostRecentMessage = messages[messages.length - 1];

      // Reset the rectangles to their default fill color
      document.querySelectorAll(".status-rectangle4, .status-rectangle5, .status-rectangle6").forEach(rect => {
        const defaultFill = rect.getAttribute("data-default-fill");
        rect.setAttribute("fill", defaultFill);
      });

      // Apply new status color based on the most recent message
      if (mostRecentMessage == 'status: idle') {
        document.querySelectorAll(".status-rectangle4").forEach(rect => {
          rect.setAttribute("fill", "blue");
        });
      } else if (mostRecentMessage == 'status: moving') {
        document.querySelectorAll(".status-rectangle5").forEach(rect => {
          rect.setAttribute("fill", "green");
        });
      } else {
        document.querySelectorAll(".status-rectangle6").forEach(rect => {
          rect.setAttribute("fill", "red");
        });
      }

      for (const message of messages) {
        const newMessage = document.createElement('li');
        newMessage.textContent = message;
        ul.appendChild(newMessage);
      }
    });
}

// Update status messages every 1 second
setInterval(updateStatusMessages, 1000);
