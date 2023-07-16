const loginElement = document.querySelector('#login-form');
const contentElement = document.querySelector("#content-sign-in");
const userDetailsElement = document.querySelector('#user-details');
const authBarElement = document.querySelector("#authentication-bar");

// Elements for sensor readings
const hrElement = document.getElementById("heartRate");
const spo2Element = document.getElementById("spo2");

// MANAGE LOGIN/LOGOUT UI
const setupUI = (user) => {
  if (user) {
    //toggle UI elements
    loginElement.style.display = 'none';
    contentElement.style.display = 'block';
    authBarElement.style.display ='block';
    userDetailsElement.style.display ='block';
    userDetailsElement.innerHTML = user.email;

    // get user UID to get data from database
    var uid = user.uid;
    console.log(uid);

    // Database paths (with user UID)
    var dbPathHR = 'UsersData/' + uid.toString() + '/Heart Rate';
    var dbPathSpO2 = 'UsersData/' + uid.toString() + '/SPO2';

    // Database references
    var dbRefHR = firebase.database().ref().child(dbPathHR);
    var dbRefSpO2 = firebase.database().ref().child(dbPathSpO2);

    // Update page with new readings
    dbRefHR.on('value', snap => {
      hrElement.innerText = snap.val().toFixed(2);
    });

    dbRefSpO2.on('value', snap => {
      spo2Element.innerText = snap.val().toFixed(2);
    });

  // if user is logged out
  } else{
    // toggle UI elements
    loginElement.style.display = 'block';
    authBarElement.style.display ='none';
    userDetailsElement.style.display ='none';
    contentElement.style.display = 'none';
  }
}