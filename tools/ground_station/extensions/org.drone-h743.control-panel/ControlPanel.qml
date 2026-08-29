import QtQuick
import QtQuick.Controls
import SerialStudio

//
// Proof-of-concept interactive widget for drone-H743.
//
// The point of this package is to demonstrate, on real hardware, the one thing that
// decides the whole ground-station architecture: a widget extension can accept user
// input and WRITE to the flight controller, in a stock GPLv3 build, with no fork.
//
// It works because extensions are required to `import SerialStudio` (that is where
// ExtensionDataModel lives), and ApiTerminalBridge is registered into that very same
// QML module, unconditionally, above the BUILD_COMMERCIAL block in
// ModuleManager::registerQmlTypes(). So the import that every extension already needs
// also brings in the full in-process command gateway.
//
Item {
  id: root

  required property color color
  required property string widgetId
  required property var windowRoot
  required property ExtensionDataModel model

  // The command gateway. run() takes "scope.verb {json}" and dispatches it through
  // CommandHandler with Trusted origin, returning {ok, result | error, errorCode}.
  ApiTerminalBridge {
    id: api
  }

  readonly property string paramName: model.config["paramName"] || "coax.roll_angle_kp"
  readonly property real paramMax: model.config["paramMax"] || 10.0

  property string lastLine: "(idle)"
  property bool lastOk: true

  //
  // Sends one raw line to the flight controller.
  //
  // io.writeData is used rather than console.send, despite console.send being the
  // documented choice for text protocols. Reason, found by measuring against a
  // loopback TCP server:
  //
  //   console.send returns {ok: true, sent: true} EVEN WHEN THE LINK IS DOWN.
  //   io.writeData fails honestly with {errorCode: EXECUTION_ERROR,
  //   category: "connection_lost", message: "Not connected"}.
  //
  // A button that reports "sent" while nothing left the machine is worse than one
  // that reports nothing, so the honest call wins and the line ending is appended
  // here instead. Both were verified to put the exact bytes on the wire.
  //
  // io.writeData takes base64, so the line is encoded before transmission.
  //
  function sendLine(line) {
    const payload = Qt.btoa(line + "\r\n")
    const response = api.run("io.writeData " + JSON.stringify({ data: payload }))
    root.lastOk = response.ok === true
    root.lastLine = root.lastOk
        ? ("TX  " + line)
        : ("ERR " + (response.error || "unknown") + " [" + (response.errorCode || "-") + "]")
    return response
  }

  Column {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 10

    Label {
      text: root.model.title + "  —  H743 Control Panel"
      color: root.color
      font.bold: true
      elide: Text.ElideRight
      width: parent.width
    }

    // ---- Query / safe commands -------------------------------------------------
    // Only read-only and non-actuator commands live here on purpose: this package
    // exists to prove the write path, not to command an aircraft. Anything that
    // moves an effector belongs behind the firmware Action contract, never behind
    // a bare button.
    Flow {
      width: parent.width
      spacing: 6

      Repeater {
        model: ["PING", "STATUS?", "TELEM?", "PARAM?", "PID?", "RTOS?"]
        delegate: Button {
          required property string modelData
          text: modelData
          onClicked: root.sendLine(modelData)
        }
      }
    }

    Rectangle {
      width: parent.width
      height: 1
      color: root.color
      opacity: 0.3
    }

    // ---- Live parameter slider -------------------------------------------------
    // This is the pattern the PID tuner needs and the one thing a stock Serial
    // Studio Action cannot do: an Action's txData is a fixed string with no
    // placeholder substitution (see DataModel::get_tx_bytes), so a value that the
    // operator chooses at runtime can only be sent from a widget like this one.
    Label {
      text: "Parameter: " + root.paramName
      color: root.color
      elide: Text.ElideRight
      width: parent.width
    }

    Row {
      width: parent.width
      spacing: 8

      Slider {
        id: paramSlider
        width: parent.width - readout.width - 16
        from: 0.0
        to: root.paramMax
        stepSize: root.paramMax / 200.0

        // Only transmit when the operator releases the handle. Streaming a write on
        // every pixel of drag would flood the link and, on a 57600 baud telemetry
        // radio, starve the telemetry stream itself.
        onPressedChanged: {
          if (!pressed)
            root.sendLine("PARAM " + root.paramName + "=" + value.toFixed(4))
        }
      }

      Label {
        id: readout
        text: paramSlider.value.toFixed(4)
        color: root.color
        width: 70
        horizontalAlignment: Text.AlignRight
      }
    }

    Button {
      text: "Read back (PARAM?)"
      onClicked: root.sendLine("PARAM?")
    }

    Rectangle {
      width: parent.width
      height: 1
      color: root.color
      opacity: 0.3
    }

    // ---- Result of the last bridge call ---------------------------------------
    Label {
      width: parent.width
      wrapMode: Text.Wrap
      text: root.lastLine
      color: root.lastOk ? root.color : "#d05050"
    }

    Label {
      width: parent.width
      wrapMode: Text.Wrap
      opacity: 0.7
      color: root.color
      text: "datasets=" + root.model.datasetCount
            + "  first=" + root.model.text
    }
  }
}
