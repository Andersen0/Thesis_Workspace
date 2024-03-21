function fetchDataAndUpdateDisplay(endpoint, elementId) {
    fetch(endpoint)
        .then(response => response.json())
        .then(data => {
            document.getElementById(elementId).innerText = JSON.stringify(data);
        })
        .catch(error => console.error(`Error fetching data from ${endpoint}:`, error));
}

window.onload = function() {
    fetchDataAndUpdateDisplay('/dtt', 'dttDisplay');
    fetchDataAndUpdateDisplay('/classifier', 'classifierDisplay'); // Make sure to have an element with id="classifierDisplay" in your HTML
    // Add more calls as necessary
};