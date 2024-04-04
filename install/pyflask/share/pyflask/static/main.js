// Store the most recent status message
let lastStatusMessage = null;

// Store the status messages and topic messages
let statusData = [];

// Store the data from Yolo and distance sensor
var lastClass, lastDistance, state;

function updateStatusMessages() {

  updateTable();
  lastStatusMessage = state;
  console.log(state);
  // Only update the status circle color if a new message has been received
  if (state == 'idle') {
    document.getElementById('status-circle').style.backgroundColor = 'blue';
  } else if (state == 'moving') {
    document.getElementById('status-circle').style.backgroundColor = 'green';
  } else {
    document.getElementById('status-circle').style.backgroundColor = 'red';
  }
}

// Update status messages every 1 second
setInterval(updateStatusMessages, 1000);


// Update the table
function updateTable() {
  Promise.all([
      fetch('/classes').then(response => response.json())
  ]).then(([classes]) => {
      var lastClass;
      if (classes[0] === "none"){
          lastClass = "none";
          lastDistance = "none";
      } else {
          lastClass = classes[0][0] === 0 ? 'adult' : 'worker';
          console.log(lastClass);
          var lastDistance = classes[0][4]; 
      }


      var cellId = lastDistance + '_' + lastClass;
      var cell = document.getElementById(cellId);

      // Reset all cells
      var cells = document.querySelectorAll('td');
      cells.forEach(cell => cell.classList.remove('active'));

      if (cell) {
          cell.classList.add('active');
          state = cell.textContent.trim().toLowerCase();
          localStorage.setItem('state', JSON.stringify(state));
          window.globalState = state;

          // Now send the state to the Python backend
          fetch('/receive_data', {
              method: 'POST',
              headers: {
                  'Content-Type': 'application/json'
              },
              body: JSON.stringify({
                  state: state
              })
          }).then(response => response.json()).then(data => {
              console.log(data.message);
          });
      }
  });
}
updateTable();  // Call this function initially


function updateMessages() {
    // We'll assume the most recent message is at the end of the array
    if (statusData.length > 30) {
      statusData.shift();
    }
    // Determine numerical value for status
    let statusValue;
    if (state === 'idle') {
      statusValue = 1;
    } else if (state === 'moving') {
      statusValue = 2;
    } else {
      statusValue = 0;  // Assume any other status is 'stopping'
    }


    // Update the status graph
    updateStatusMessages();
    updateTable();
  };


// Update messages every 1 second
setInterval(updateMessages, 1000);
// Call choicematrix() function every 1000ms (1 second)
// setInterval(choicematrix, 1000);

// Toggle buttons to show/hide elements on the page
document.getElementById('toggleButton').onclick = function() {
  var circle = document.getElementById('status-circle');
  if (circle.style.display === "none") {
    circle.style.display = "block";
  } else {
    circle.style.display = "none";
  }
}

// Button to send a shutdown request to the server
document.getElementById('shutdownButton').onclick = function() {
  fetch('/shutdown', {method: 'POST'})
    .then(response => {
      if (!response.ok) {
        throw new Error(`Server returned status ${response.status}`);
      }
    })
    .catch(console.error);
}

document.getElementById('toggleStatusButton').onclick = function() {
  var statusMessages = document.getElementById('recent_status_messages');
  if (statusMessages.style.display === "none") {
    statusMessages.style.display = "block";
  } else {
    statusMessages.style.display = "none";
  }
}

document.getElementById('toggleTopicButton').onclick = function() {
  var topicMessages = document.getElementById('recent_topic_messages');
  if (topicMessages.style.display === "none") {
    topicMessages.style.display = "block";
  } else {
    topicMessages.style.display = "none";
  }
}

document.getElementById('toggleImageButton').onclick = function() {
  var image = document.getElementById('videoElement');
  if (image.style.display === "none") {
    image.style.display = "";  // Show the image
  } else {
    image.style.display = "none";  // Hide the image
  }
}

// // Toggle status-chart from html
// document.getElementById('toggleStatusChartButton').onclick = function() {
//   var statusChart = document.getElementById('status-area');
//   if (statusChart.style.display === "none") {
//     statusChart.style.display = "block";
//   } else {
//     statusChart.style.display = "none";
//   }
// }

// Click event handler for the buttons
function handleClick(buttonLabel) {
  const visualisation = d3.select("#visualisation");

  if (buttonLabel === "Robot") {
    // Toggle visibility of the visualisation SVG
    const isHidden = visualisation.style("display") === "none";
    visualisation.style("display", isHidden ? "block" : "none");
  } else {
    // Handle other button clicks here
  }
}
