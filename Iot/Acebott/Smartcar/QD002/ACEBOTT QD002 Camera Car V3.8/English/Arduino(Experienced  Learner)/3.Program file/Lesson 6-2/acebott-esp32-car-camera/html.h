const char* html PROGMEM = R"HTMLHOMEPAGE(

<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />

    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>ESP32 CAM</title>
    <style>
      * {
        padding: 0px;
        margin: 0px;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
      }
      .grid-container {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        grid-template-rows: repeat(4, 1fr);
        gap: 10px;
      }
      .grid-item {
        display: flex;
        justify-content: center;
      }
      button {
        background-color: #0d6efd;
        border: none;
        color: white;
        padding: 8px 16px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 12px;
        margin: 6px 3px;
        cursor: pointer;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }

      button:hover {
        background-color: #3778e7;
      }
    </style>
  </head>
  <body>
    <img
      id="camera"
      style="background-color: black"
      width="100%"
      height="200"
    />
    <div style="display: flex; justify-content: space-around">
      <button onclick="camera(this)" id="StartScreen">Start Screen</button>
      <button onclick="camera(this)" id="PauseScreen">Pause Screen</button>
      <button onclick="camera(this)" id="CloseScreen">Close Screen</button>
      <button type="button" id="ESP32LED">Open LED</button>
    </div>
    <hr />
    <div style="display: flex; justify-content: center">
      <div class="grid-container">
        <div id="LeftUp" class="grid-item"><button>Left Up</button></div>
        <div id="Forward" class="grid-item"><button>Forward</button></div>
        <div id="RightUp" class="grid-item"><button>Right Up</button></div>
        <div id="Left" class="grid-item"><button>Left</button></div>
        <div class="grid-item"></div>
        <div id="Right" class="grid-item"><button>Right</button></div>
        <div id="LeftDown" class="grid-item"><button>Left Down</button></div>
        <div id="Backward" class="grid-item"><button>Backward</button></div>
        <div id="RightDown" class="grid-item"><button>Right Down</button></div>

        <div class="grid-item">
          <button id="Anticlockwise">Anticlockwise</button>
        </div>
        <div class="grid-item"></div>
        <div class="grid-item"><button id="Clockwise">Clockwise</button></div>
      </div>
    </div>
    <div>
      <span>Speed</span>
      <input
        id="speed"
        type="range"
        max="5"
        min="1"
        step="1"
        value="3"
        style="width: 80%; margin-bottom: 20px; margin-top: 10px"
      />
    </div>
    <div>
      <span>Servo</span>
      <input
        id="servo"
        type="range"
        max="180"
        step="30"
        style="width: 80%; margin-bottom: 20px; margin-top: 10px"
      />
    </div>
    <hr />
    <!-- <div style="display: flex; justify-content: space-around">
      <button onclick="general('LED&value=1')">Open LED</button>
      <button onclick="general('LED&value=0')">Close LED</button>
      <button onclick="mode(this)" id="Shooting">Shooting</button>
    </div>
    <hr /> -->
    <div style="display: flex; justify-content: space-around">
      <button onclick="general('Buzzer&value=1')">Buzzer1</button>
      <button onclick="general('Buzzer&value=2')">Buzzer2</button>
      <button onclick="general('Buzzer&value=3')">Buzzer3</button>
      <button onclick="general('Buzzer&value=4')">Buzzer4</button>
    </div>
    <hr />
    <div style="display: flex; justify-content: space-around">
      <button onclick="general('Track&value=1')">Track1</button>
      <button onclick="general('Track&value=2')">Track2</button>
      <button onclick="mode(this)" id="Avoidance">Avoidance</button>
      <button onclick="mode(this)" id="Follow">Follow</button>
      <button onclick="general('stopA')">Stop</button>
    </div>
    <hr />

    <script>
      window.mobileCheck = function () {
        let check = false;
        (function (a) {
          if (
            /(android|bb\d+|meego).+mobile|avantgo|bada\/|blackberry|blazer|compal|elaine|fennec|hiptop|iemobile|ip(hone|od)|iris|kindle|lge |maemo|midp|mmp|mobile.+firefox|netfront|opera m(ob|in)i|palm( os)?|phone|p(ixi|re)\/|plucker|pocket|psp|series(4|6)0|symbian|treo|up\.(browser|link)|vodafone|wap|windows ce|xda|xiino/i.test(
              a
            ) ||
            /1207|6310|6590|3gso|4thp|50[1-6]i|770s|802s|a wa|abac|ac(er|oo|s\-)|ai(ko|rn)|al(av|ca|co)|amoi|an(ex|ny|yw)|aptu|ar(ch|go)|as(te|us)|attw|au(di|\-m|r |s )|avan|be(ck|ll|nq)|bi(lb|rd)|bl(ac|az)|br(e|v)w|bumb|bw\-(n|u)|c55\/|capi|ccwa|cdm\-|cell|chtm|cldc|cmd\-|co(mp|nd)|craw|da(it|ll|ng)|dbte|dc\-s|devi|dica|dmob|do(c|p)o|ds(12|\-d)|el(49|ai)|em(l2|ul)|er(ic|k0)|esl8|ez([4-7]0|os|wa|ze)|fetc|fly(\-|_)|g1 u|g560|gene|gf\-5|g\-mo|go(\.w|od)|gr(ad|un)|haie|hcit|hd\-(m|p|t)|hei\-|hi(pt|ta)|hp( i|ip)|hs\-c|ht(c(\-| |_|a|g|p|s|t)|tp)|hu(aw|tc)|i\-(20|go|ma)|i230|iac( |\-|\/)|ibro|idea|ig01|ikom|im1k|inno|ipaq|iris|ja(t|v)a|jbro|jemu|jigs|kddi|keji|kgt( |\/)|klon|kpt |kwc\-|kyo(c|k)|le(no|xi)|lg( g|\/(k|l|u)|50|54|\-[a-w])|libw|lynx|m1\-w|m3ga|m50\/|ma(te|ui|xo)|mc(01|21|ca)|m\-cr|me(rc|ri)|mi(o8|oa|ts)|mmef|mo(01|02|bi|de|do|t(\-| |o|v)|zz)|mt(50|p1|v )|mwbp|mywa|n10[0-2]|n20[2-3]|n30(0|2)|n50(0|2|5)|n7(0(0|1)|10)|ne((c|m)\-|on|tf|wf|wg|wt)|nok(6|i)|nzph|o2im|op(ti|wv)|oran|owg1|p800|pan(a|d|t)|pdxg|pg(13|\-([1-8]|c))|phil|pire|pl(ay|uc)|pn\-2|po(ck|rt|se)|prox|psio|pt\-g|qa\-a|qc(07|12|21|32|60|\-[2-7]|i\-)|qtek|r380|r600|raks|rim9|ro(ve|zo)|s55\/|sa(ge|ma|mm|ms|ny|va)|sc(01|h\-|oo|p\-)|sdk\/|se(c(\-|0|1)|47|mc|nd|ri)|sgh\-|shar|sie(\-|m)|sk\-0|sl(45|id)|sm(al|ar|b3|it|t5)|so(ft|ny)|sp(01|h\-|v\-|v )|sy(01|mb)|t2(18|50)|t6(00|10|18)|ta(gt|lk)|tcl\-|tdg\-|tel(i|m)|tim\-|t\-mo|to(pl|sh)|ts(70|m\-|m3|m5)|tx\-9|up(\.b|g1|si)|utst|v400|v750|veri|vi(rg|te)|vk(40|5[0-3]|\-v)|vm40|voda|vulc|vx(52|53|60|61|70|80|81|83|85|98)|w3c(\-| )|webc|whit|wi(g |nc|nw)|wmlb|wonu|x700|yas\-|your|zeto|zte\-/i.test(
              a.substr(0, 4)
            )
          )
            check = true;
        })(navigator.userAgent || navigator.vendor || window.opera);
        return check;
      };

      document.querySelector("#servo").addEventListener("input", (event) => {
        fetch(
          `/control?cmd=${event.currentTarget.id}&angle=${event.target.value}`
        ).then((response) => console.log(response.statusText));
        // document.getElementById("info").innerHTML = event.target.value;
      });

      document.querySelector("#speed").addEventListener("input", (event) => {
        fetch(
          `/control?cmd=${event.currentTarget.id}&value=${event.target.value}`
        ).then((response) => console.log(response.statusText));

        // document.getElementById("info").innerHTML = event.target.value;
      });

      let direction = [
        "LeftUp",
        "Forward",
        "RightUp",
        "Left",
        "Right",
        "LeftDown",
        "Backward",
        "RightDown",
        "Anticlockwise",
        "Clockwise",
      ];
      direction.forEach((item) => {
        let element = document.getElementById(item);
        element.addEventListener(
          window.mobileCheck() ? "touchstart" : "mousedown",
          (event) => {
            fetch(`/control?cmd=car&direction=${event.currentTarget.id}`).then(
              (response) => console.log(response.statusText)
            );
          }
        );
        element.addEventListener(
          window.mobileCheck() ? "touchend" : "mouseup",
          (event) => {
            fetch(`/control?cmd=car&direction=stop`).then((response) =>
              console.log(response.statusText)
            );
          }
        );
      });

      var button = document.getElementById("ESP32LED");
            var buttonText = ["Close LED", "Open LED"];
            var currentTextIndex = 0;
            button.addEventListener("click", function() {
              button.textContent = buttonText[currentTextIndex];
              if (button.textContent === "Close LED") {
                general('CAM_LED&value=1')
              } else if (button.textContent === "Open LED") {
                general('CAM_LED&value=0')
              }
              currentTextIndex = (currentTextIndex + 1) % buttonText.length;
            });

      function mode(e) {
        console.log(event.currentTarget.id);
        fetch(`/control?cmd=${event.currentTarget.id}`).then((response) =>
          console.log(response.statusText)
        );
      }

      function camera(e) {
        var img = document.getElementById("camera");
        switch (event.currentTarget.id) {
          case "StartScreen":
            img.src = "http://192.168.4.1/Stream";
            break;
          case "PauseScreen":
            img.src = "http://192.168.4.1/capture";
            break;
          case "CloseScreen":
            img.src = `data:image/jpeg;base64,/9j/4AAQSkZJRgABAQEAYABgAAD/4QAiRXhpZgAATU0AKgAAAAgAAQESAAMAAAABAAEAAAAAAAD/2wBDAAIBAQIBAQICAgICAgICAwUDAwMDAwYEBAMFBwYHBwcGBwcICQsJCAgKCAcHCg0KCgsMDAwMBwkODw0MDgsMDAz/2wBDAQICAgMDAwYDAwYMCAcIDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAz/wAARCAAeAB4DASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD+f+iiigAooooAKKKKACiiigD/2Q==`;
            break;
        }
      }

      function general(data) {
        fetch(`/control?cmd=${data}`).then((response) =>
          console.log(response.statusText)
        );
      }
    </script>
  </body>
</html>


)HTMLHOMEPAGE";
