// Save new user or load existing user information
function loginUser() {
    var usrEmail = document.getElementById("userEmail").value;

    // If the username is an empty string,
    // or contains characters other than letters, numbers, and the symbols @.-
    // or if it contains more than one @ sign,
    // decline the email and refuse to login the user.
    if (!usrEmail ||
        /[^a-zA-Z0-9@.-]/.test(usrEmail) ||
        /^(?![^@]*@[^@]*$)/.test(usrEmail)) 
    {
        alert("Invalid email address.");
        return;
    }

    // find user in DB by given email
    findUserByEmail(usrEmail).then (function (user) {
        
        // user not found, redirect to register page.
        if (!user) {
            alert("User with email: " + usrEmail + ", does not exist." +
            "Please register.");
            redirectToRegister();
        }

        // if the user was found,
        // save his data in the local storage and
        // redirect him to the main page of the web client.
        console.log("User with email: " + user.email + 
            ", Has the name: " + user.name +
            ", Leg length: " + user.legLengthCm +
            ", Gender: " + user.gender,
            " and weighs: " + user.weightKg,
            ". Recommended seat height is: " + user.recommendedSeatHeightMm,
            ", Preferred seat height is: " + user.preferredSeatHeightMm);
        localStorage.setItem("activeUserName", user.name);
        localStorage.setItem("activeUserEmail", user.email);
        localStorage.setItem("activeUserLegLength", user.legLengthCm);
        localStorage.setItem("activeUserGender", user.gender);
        localStorage.setItem("activeUserWeightKg", user.weightKg);
        localStorage.setItem("activeUserRecommendedSeatHeight", user.recommendedSeatHeightMm);
        localStorage.setItem("activeUserPreferredSeatHeight", user.preferredSeatHeightMm);
        redirectToWebClient(user.email);
    });
}

// Redirect user to web client page
function redirectToWebClient(usrEmail) {
    document.location.href='EbmsWebClient.html';
}

// Redirect user to register page
function redirectToRegister() {
    document.location.href='EbmsRegisterPage.html';
}