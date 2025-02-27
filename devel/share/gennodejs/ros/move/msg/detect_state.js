// Auto-generated. Do not edit!

// (in-package move.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class detect_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.detect_state = null;
    }
    else {
      if (initObj.hasOwnProperty('detect_state')) {
        this.detect_state = initObj.detect_state
      }
      else {
        this.detect_state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type detect_state
    // Serialize message field [detect_state]
    bufferOffset = _serializer.bool(obj.detect_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type detect_state
    let len;
    let data = new detect_state(null);
    // Deserialize message field [detect_state]
    data.detect_state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'move/detect_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb4ea15efeb62557142025112b232836';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool detect_state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new detect_state(null);
    if (msg.detect_state !== undefined) {
      resolved.detect_state = msg.detect_state;
    }
    else {
      resolved.detect_state = false
    }

    return resolved;
    }
};

module.exports = detect_state;
