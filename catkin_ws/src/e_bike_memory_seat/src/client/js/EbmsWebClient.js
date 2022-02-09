validateUser();

function validateUser() {
    let userName = localStorage.getItem("activeUserName");
    if (!userName) {
        alert("Please login before using the EBMS Web App.");
        document.location.href = 'EbmsLoginPage.html';
    }
    
    // Greet the user when the html body has loaded.
    document.getElementById("usrGreeting").innerHTML = userName;
}

function changeDivContentAdjustmentType() {
    if (document.getElementById("autoRadio").checked) {
        // Display automatic seat adjustment form. User must enter his height and weight.
        document.getElementById("variableContentDiv").innerHTML = 
           `<form id="autoForm">
                <p>
                    <span id="usrNameInAF"></span>, you have stated that your inseam length is
                    <span id="legLengthInAF"></span>cm long. Therefor we recommend,
                    a seat height of <span id="recommendedSeatHeightInAF"></span>mm.
                </p>
                <button type="submit">Order E-Bike</button>
            </form>`;
        document.getElementById("usrNameInAF").innerHTML =
            localStorage.getItem("activeUserName");
        document.getElementById("legLengthInAF").innerHTML = 
            localStorage.getItem("activeUserLegLength");

        calculateSeatHeight();

        document.getElementById("recommendedSeatHeightInAF").innerHTML = 
            localStorage.getItem("activeUserRecommendedSeatHeight");
        
        document.getElementById("autoForm").addEventListener("submit", handleAutoSubmitBtn);
    }
    else {
        // Display manual seat adjustment form. User directly specifies his desired seat height.
        document.getElementById("variableContentDiv").innerHTML =
            `<form id="manForm">
                <p>Enter your desired seat height: <input type="number" id="usrInput" min="10" max="140"> <i>(Seat height can be adjusted from 10 to 140mm)</i></p>
                <button type="submit">Order E-Bike</button>
            </form>`;
            document.getElementById("manForm").addEventListener("submit", handleManSubmitBtn);
    }
}

function calculateSeatHeight() {
    const usrLegLength = parseInt(localStorage.getItem("activeUserLegLength"));
    const lemondConstant = 883;
    var calculatedSeatHeight = Math.round((usrLegLength * lemondConstant)/1000);

    // The seat cannot be lifted higher than 140mm or bellow 10mm.
    if (calculatedSeatHeight > 140) {
        calculatedSeatHeight = 140;
    }
    else if (calculatedSeatHeight < 10) {
        calculatedSeatHeight = 10;
    }
    
    localStorage.setItem("activeUserRecommendedSeatHeight", calculatedSeatHeight);
}

function handleAutoSubmitBtn(event) {
    event.preventDefault();
    const recommendedSeatHeight = parseInt(localStorage.getItem("activeUserRecommendedSeatHeight"));
    if (!recommendedSeatHeight) {
        console.error("Invalid recommendedSeatHeight=", recommendedSeatHeight);
        return;
    }
    sendActionToROS(recommendedSeatHeight);
}

function handleManSubmitBtn(event) {
    event.preventDefault();
    const usrInput = parseInt(document.getElementById("usrInput").value);
    console.log("usrInput type: ", typeof(usrInput));
    console.log("usrInput is: ", usrInput);
    if (Number.isNaN(usrInput)) {
        alert("Invalid data!");
        return;
    }
    sendActionToROS(usrInput);
}

function sendActionToROS(wantedSeatHeight) {
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    var ebmsActionClient = new ROSLIB.ActionClient({
        ros : ros,
        serverName : '/ebmsRosNode',
        actionName : 'e_bike_memory_seat/adjustSeatHeightAction'
    })

    var goal = new ROSLIB.Goal({
        actionClient : ebmsActionClient,
        goalMessage : {
            wantedHeight : wantedSeatHeight
        }
    });

    goal.on('feedback', function(feedback){
        document.getElementById("currentHeightOutput").innerHTML = 
            "The current seat height is: " + feedback.currentHeight + "mm";
        console.log('Feedback: ' + feedback.currentHeight);
    });

    goal.on('result', function(result){
        document.getElementById("currentHeightOutput").innerHTML = 
            "Seat has been adjusted to: " + result.finalHeight + "mm";
        console.log('Final result: ' + result.finalHeight);
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function(error) {
        console.log('Connection to websocket server closed.');
    });

    goal.send();
}