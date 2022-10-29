package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {
  private Gamepad _gamepad;
  private boolean _isAPressed = false;
  private boolean _isBPressed = false;
  private boolean _isXPressed = false;
  private boolean _isYPressed = false;
  private boolean _isDPressed = false;
  private boolean _isLPressed = false;
  private boolean _isRPressed = false;
  private boolean _isUPressed = false;
  private EventHandlerManager aPressedEventHandlerManager;
  private EventHandlerManager bPressedEventHandlerManager;
  private EventHandlerManager xPressedEventHandlerManager;
  private EventHandlerManager yPressedEventHandlerManager;
  private EventHandlerManager aReleasedEventHandlerManager;
  private EventHandlerManager bReleasedEventHandlerManager;
  private EventHandlerManager xReleasedEventHandlerManager;
  private EventHandlerManager yReleasedEventHandlerManager;
  private EventHandlerManager dPressedEventHandlerManager;
  private EventHandlerManager lPressedEventHandlerManager;
  private EventHandlerManager rPressedEventHandlerManager;
  private EventHandlerManager uPressedEventHandlerManager;
  private EventHandlerManager dReleasedEventHandlerManager;
  private EventHandlerManager lReleasedEventHandlerManager;
  private EventHandlerManager rReleasedEventHandlerManager;
  private EventHandlerManager uReleasedEventHandlerManager;

  public GamepadWrapper() {
    this.aPressedEventHandlerManager = new EventHandlerManager();
    this.bPressedEventHandlerManager = new EventHandlerManager();
    this.xPressedEventHandlerManager = new EventHandlerManager();
    this.yPressedEventHandlerManager = new EventHandlerManager();
    this.aReleasedEventHandlerManager = new EventHandlerManager();
    this.bReleasedEventHandlerManager = new EventHandlerManager();
    this.xReleasedEventHandlerManager = new EventHandlerManager();
    this.yReleasedEventHandlerManager = new EventHandlerManager();
    this.dPressedEventHandlerManager = new EventHandlerManager();
    this.lPressedEventHandlerManager = new EventHandlerManager();
    this.rPressedEventHandlerManager = new EventHandlerManager();
    this.uPressedEventHandlerManager = new EventHandlerManager();
    this.dReleasedEventHandlerManager = new EventHandlerManager();
    this.lReleasedEventHandlerManager = new EventHandlerManager();
    this.rReleasedEventHandlerManager = new EventHandlerManager();
    this.uReleasedEventHandlerManager = new EventHandlerManager();
  }

  public GamepadWrapper setGamepad(Gamepad gamepad) {
    this._gamepad = gamepad;
    return this;
  }

  public GamepadWrapper subscribeAPressedEvent(EventHandler eventHandler) {
    this.aPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeBPressedEvent(EventHandler eventHandler) {
    this.bPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeXPressedEvent(EventHandler eventHandler) {
    this.xPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeYPressedEvent(EventHandler eventHandler) {
    this.yPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeAReleasedEvent(EventHandler eventHandler) {
    this.aReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeBReleasedEvent(EventHandler eventHandler) {
    this.bReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeXReleasedEvent(EventHandler eventHandler) {
    this.xReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeYReleasedEvent(EventHandler eventHandler) {
    this.yReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeDPressedEvent(EventHandler eventHandler) {
    this.dPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeLPressedEvent(EventHandler eventHandler) {
    this.lPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeRPressedEvent(EventHandler eventHandler) {
    this.rPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeUPressedEvent(EventHandler eventHandler) {
    this.uPressedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeDReleasedEvent(EventHandler eventHandler) {
    this.dReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeLReleasedEvent(EventHandler eventHandler) {
    this.lReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeRReleasedEvent(EventHandler eventHandler) {
    this.rReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public GamepadWrapper subscribeUReleasedEvent(EventHandler eventHandler) {
    this.uReleasedEventHandlerManager.subscribe(eventHandler);
    return this;
  }

  public Gamepad getGamepad() {
    return this._gamepad;
  }

  public boolean isAPressed() {
    return this._isAPressed;
  }

  public boolean isBPressed() {
    return this._isBPressed;
  }

  public boolean isXPressed() {
    return this._isXPressed;
  }

  public boolean isYPressed() {
    return this._isYPressed;
  }

  public boolean isDPressed() {
    return this._isDPressed;
  }

  public boolean isLPressed() {
    return this._isLPressed;
  }

  public boolean isRPressed() {
    return this._isRPressed;
  }

  public boolean isUPressed() {
    return this._isUPressed;
  }

  private void updateLeftButtons() {
    boolean nowDPressed = this._gamepad.dpad_down;
    boolean nowLPressed = this._gamepad.dpad_left;
    boolean nowRPressed = this._gamepad.dpad_right;
    boolean nowUPressed = this._gamepad.dpad_up;
    if (!this._isDPressed && nowDPressed) this.dPressedEventHandlerManager.execute();
    if (!this._isLPressed && nowLPressed) this.lPressedEventHandlerManager.execute();
    if (!this._isRPressed && nowRPressed) this.rPressedEventHandlerManager.execute();
    if (!this._isUPressed && nowUPressed) this.uPressedEventHandlerManager.execute();
    if (this._isDPressed && !nowDPressed) this.dReleasedEventHandlerManager.execute();
    if (this._isLPressed && !nowLPressed) this.lReleasedEventHandlerManager.execute();
    if (this._isRPressed && !nowRPressed) this.rReleasedEventHandlerManager.execute();
    if (this._isUPressed && !nowUPressed) this.uReleasedEventHandlerManager.execute();
    this._isDPressed = nowDPressed;
    this._isLPressed = nowLPressed;
    this._isRPressed = nowRPressed;
    this._isUPressed = nowUPressed;
  }

  private void updateRightButtons() {
    boolean nowAPressed = this._gamepad.a;
    boolean nowBPressed = this._gamepad.b;
    boolean nowXPressed = this._gamepad.x;
    boolean nowYPressed = this._gamepad.y;
    if (!this._isAPressed && nowAPressed) this.aPressedEventHandlerManager.execute();
    if (!this._isBPressed && nowBPressed) this.bPressedEventHandlerManager.execute();
    if (!this._isXPressed && nowXPressed) this.xPressedEventHandlerManager.execute();
    if (!this._isYPressed && nowYPressed) this.yPressedEventHandlerManager.execute();
    if (this._isAPressed && !nowAPressed) this.aReleasedEventHandlerManager.execute();
    if (this._isBPressed && !nowBPressed) this.bReleasedEventHandlerManager.execute();
    if (this._isXPressed && !nowXPressed) this.xReleasedEventHandlerManager.execute();
    if (this._isYPressed && !nowYPressed) this.yReleasedEventHandlerManager.execute();
    this._isAPressed = nowAPressed;
    this._isBPressed = nowBPressed;
    this._isXPressed = nowXPressed;
    this._isYPressed = nowYPressed;
  }
  
  public void update() {
    this.updateLeftButtons();
    this.updateRightButtons();
  }
}