import GstWebRTCAPI from './gstwebrtc-api-3.0.0.esm.js';
// import AFRAME from './aframe-v1.7.1.module.min.js';

// Wait for A-Frame scene to load
AFRAME.registerComponent('video-component', {
    init: function () {
        // Controllers are enabled
        this.initElements();
        this.initState();
        this.initWebSocket(); // 初始化WebSocket（但不强制连接）

        const signalingProtocol = window.location.protocol.startsWith("https") ? "wss" : "ws";
        const gstWebRTCConfig = {
            meta: { name: `WebClient-${Date.now()}` },
            signalingServerUrl: `${signalingProtocol}://${window.location.hostname}/webrtc`,
            // signalingServerUrl: `wss://192.168.41.31/webrtc`,
        };

        this.gstWebRtcApi = new GstWebRTCAPI(gstWebRTCConfig);
    },

    initWebSocket: function () {
        // 初始化WebSocket相关状态，不立即连接
        this.websocket = null;
        this.isConnected = false;
        this.videoStreamStatus = false;
        // 用于存储start/stop命令的回调Promise
        this.streamCommandResolver = null;
        this.pwsState = false;
        this.initPwsStateWebSocket();
        this.connectWebSocketAsync();
    },

    // 封装：连接WebSocket并返回Promise（解决异步等待问题）
    connectWebSocketAsync: function () {
        return new Promise((resolve, reject) => {
            if (this.isConnected) {
                resolve(true);
                return;
            }

            const protocol = 'wss:';
            const host = window.location.host;
            // const host = '192.168.41.31';
            const wsUrl = `${protocol}//${host}/jetson_ws/ws`;

            try {
                this.websocket = new WebSocket(wsUrl);

                this.websocket.onopen = (event) => {
                    this.isConnected = true;
                    this.connectVideoBtn.textContent = '断开';
                    // 连接成功后立即查询状态
                    this.sendWebSocketData({ command: 'status' })
                        .then(() => resolve(true))
                        .catch(reject);
                };

                this.websocket.onerror = (event) => {
                    this.isConnected = false;
                    this.connectVideoBtn.textContent = '连接';
                    reject(new Error('WebSocket连接失败'));
                };

                this.websocket.onclose = (event) => {
                    this.websocket = null;
                    this.isConnected = false;
                    this.videoStreamStatus = false;
                    this.connectVideoBtn.textContent = '连接';
                    // 关闭时清空未完成的Promise
                    if (this.streamCommandResolver) {
                        this.streamCommandResolver(false);
                        this.streamCommandResolver = null;
                    }
                };

                this.websocket.onmessage = (event) => {
                    const data = JSON.parse(event.data);
                    const { success, status } = data;

                    // 严格校验：仅处理 success=true 且 status 是包含 running 字段的对象
                    if (success === true && typeof status === 'object' && status !== null && 'running' in status) {
                        // 提取 running 字段（确保是布尔值）
                        const running = Boolean(status.running);

                        // 更新流状态和按钮文本
                        this.videoStreamStatus = running;
                        this.startVideoBtn.textContent = running ? '停止流' : '启动流';

                        // 如果有等待中的命令回调，执行resolve
                        if (this.streamCommandResolver) {
                            this.streamCommandResolver(running); // 回调结果为流是否运行
                            this.streamCommandResolver = null;
                        }

                        console.log('处理有效流状态消息:', { running });
                    } else {
                        // 忽略不符合结构的消息（如 status 是字符串的情况）
                        console.log('忽略非目标结构的WebSocket消息:', data);
                    }
                };
            } catch (error) {
                this.isConnected = false;
                reject(error);
            }
        });
    },

    initPwsStateWebSocket: function () {
        const protocol = 'wss:';
        const host = window.location.host;
        // const host = '192.168.41.31';
        const wsUrl = `${protocol}//${host}/nuc_ws/ws`;

        try {
            this.pwsStatewebsocket = new WebSocket(wsUrl);

            this.pwsStatewebsocket.onopen = (event) => {

            };

            this.pwsStatewebsocket.onerror = (event) => {
                this.pwsState = false;
            };

            this.pwsStatewebsocket.onclose = (event) => {
                this.pwsState = false;
                document.body.innerHTML = '';
                alert("web服务已断开，请确认服务开启后重试")
                window.location.reload();
            };

            this.pwsStatewebsocket.onmessage = (event) => {
                const data = JSON.parse(event.data);
                console.log('pwsState', event.data)
                const { status, is_setup } = data;
                if (status === 'success') {
                    this.pwsState = true;
                } else {
                    this.pwsState = false;
                }
            };
        } catch (error) {
        }
    },

    // 封装：发送WebSocket数据并返回Promise（支持等待回调结果）
    sendWebSocketData: function (data) {
        return new Promise((resolve, reject) => {
            if (!this.websocket || this.websocket.readyState !== WebSocket.OPEN) {
                reject(new Error('WebSocket未连接'));
                return;
            }

            // 如果是start/stop命令，存储resolve等待回调
            if (['start', 'stop'].includes(data.command)) {
                this.streamCommandResolver = resolve;
            } else {
                // 非流控制命令，直接resolve
                resolve(true);
            }

            this.websocket.send(JSON.stringify(data));

            // 超时保护：15秒未收到回调则拒绝
            setTimeout(() => {
                if (this.streamCommandResolver === resolve) {
                    reject(new Error('命令执行超时'));
                    this.streamCommandResolver = null;
                }
            }, 15000);
        });
    },

    initRTCPeerConnection: async function () {
        this.localPc = new RTCPeerConnection({
            iceServers: [{ urls: ["stun:stun.l.google.com:19302"] }]
        });

        this.localPc.ontrack = e => {
            const video = document.querySelector('#robot-video');
            video.srcObject = e.streams[0];
            video.onloadedmetadata = () => {
                video.play().then(async () => {
                    console.log('Playback successful');
                }).catch(e => {
                    console.error('Playback failed:', e);
                });
            };
        };

        this.localPc.onicecandidate = (event) => {
            console.log("localPc:", event.candidate, event);
            if (event.candidate) {
                this.socket.emit('candidate', {
                    toUsername: '1',
                    nowUsername: '3',
                    data: event.candidate,
                    callType: 'sender'
                });
            }
        };

        this.socket = io('https://192.168.11.68', {
            path: "/rtc",
            query: { username: '3', room: "001" },
        });

        this.socket.on('connect', async () => {
            console.log('连接成功');
            let offer = await this.localPc.createOffer({
                offerToReceiveVideo: true,
                offerToReceiveAudio: false
            });
            await this.localPc.setLocalDescription(offer);
            this.socket.emit('offer', {
                toUsername: '1',
                nowUsername: '3',
                data: offer,
                callType: 'sender'
            });
        });

        this.socket.on('connect_error', (error) => {
            console.error('Socket连接失败:', error);
        });

        this.socket.on('answer', async (res) => {
            let remoteAnswer = res.data;
            await this.localPc.setRemoteDescription(remoteAnswer);
        });
    },

    initGstWebRTCAPI: function (cb) {
        this.closeSession();

        const listener = this.listener = {
            producerAdded: (producer) => {
                const producerId = producer.id
                let session = this.session = this.gstWebRtcApi.createConsumerSession(producerId);
                if (session) {

                    session.mungeStereoHack = true;
                    session.addEventListener("error", (event) => {
                        console.error(event.message, event.error);
                    });

                    session.addEventListener("closed", (event) => {
                        console.log('session.closed')
                    });

                    session.addEventListener("streamsChanged", () => {
                        // console.log('streamsChanged');
                        if (session) {
                            const streams = session.streams;
                            if (streams.length > 0) {
                                const video = document.querySelector('#robot-video');
                                video.srcObject = streams[0];
                                video.onloadedmetadata = () => {
                                    video.play().then(async () => {
                                        cb();
                                        console.log('Playback successful');
                                    }).catch(e => {
                                        console.error('Playback failed:', e);
                                    });
                                };

                                const desktopVideo = document.querySelector('#desktop-video');
                                desktopVideo.srcObject = streams[0];
                                desktopVideo.onloadedmetadata = () => {
                                    desktopVideo.play().then(async () => {
                                        console.log('Desktop video playback successful');
                                    }).catch(e => {
                                        console.error('Desktop video playback failed:', e);
                                    });
                                };
                            }
                        }
                    });

                    session.addEventListener("remoteControllerChanged", () => {
                        console.log('remoteControllerChanged');
                    });

                    session.addEventListener("clientConsumerAdded", (event) => {
                        console.info(`client consumer added: ${event?.detail?.peerId}`);
                    });

                    session.addEventListener("clientConsumerRemoved", (event) => {
                        console.info(`client consumer removed: ${event?.detail?.peerId}`);
                    });

                    session.addEventListener("stateChanged", (event) => {
                        // console.log('stateChanged', event.target.state);
                    });

                    session.connect();
                }
            },

            producerRemoved: (producer) => {
                if (this.session) {
                    console.log('producerRemoved', producer.id)
                    this.session.close();
                    this.session = null;
                }
            }
        };

        this.gstWebRtcApi.registerPeerListener(listener);
        // this.gstWebRtcApi.registerConnectionListener({
        //     connected() {
        //         console.log('GstWebRTCAPI connected')
        //     },
        //     disconnected() {
        //         console.log('GstWebRTCAPI disconnected')
        //     }
        // });

        for (const producer of this.gstWebRtcApi.getAvailableProducers()) {
            listener.producerAdded(producer);
        }
    },

    closeSession: function () {
        if (this.session) {
            this.session.close();
            this.session = null;
        }
        if (this.listener) {
            this.gstWebRtcApi.unregisterPeerListener(this.listener);
            this.listener = null;
        }
    },

    initElements: function () {
        this.leftHand = document.querySelector('#leftHand');
        this.rightHand = document.querySelector('#rightHand');
        this.videoA = document.querySelector('#robot-a-video');
        this.connectVideoBtn = document.querySelector('#connectVideoBtn');
        this.startVideoBtn = document.querySelector('#startVideoBtn');
    },

    initState: function () {
        this.leftGripDown = false;
        this.rightGripDown = false;
        this.startVideoPos = null;
        this.startControllerPos = null;
        this.session = null;
        this.isVideoVisible = false;
        this.isConnected = false;
        this.videoStreamStatus = false;
        this.connectingStatus = false;
        this.listener = null;

        // 左手柄 grip 事件
        this.leftHand.addEventListener('gripdown', (evt) => {
            this.leftGripDown = true;
        });
        this.leftHand.addEventListener('gripup', (evt) => {
            this.leftGripDown = false;
        });

        // 右手柄 grip 事件
        this.rightHand.addEventListener('gripdown', (evt) => {
            this.rightGripDown = true;
        });
        this.rightHand.addEventListener('gripup', (evt) => {
            this.rightGripDown = false;
        });

        this.leftHand.addEventListener('buttondown', async (event) => {
            const id = event.detail.id;
            if (id === 5) {
                try {
                    const videoA = document.querySelector('#robot-a-video');
                    const isVisible = videoA.getAttribute('visible');

                    if (isVisible) {
                        // 隐藏视频：关闭session
                        this.closeSession();
                        videoA.setAttribute('visible', false);
                        this.connectingStatus = false;
                    } else {
                        if (this.connectingStatus) {
                            return;
                        }
                        this.connectingStatus = true;
                        if (!this.pwsState) {
                            // alert('Jetson包未setup成功');
                            return;
                        }
                        // 1. 如果未连接WebSocket，先连接（等待连接成功）
                        if (!this.isConnected) {
                            await this.connectWebSocketAsync();
                            // console.log('WebSocket连接成功');
                        }

                        // 2. 如果视频流未启动，发送启动命令（等待回调确认启动）
                        if (!this.videoStreamStatus) {
                            const streamStarted = await this.sendWebSocketData({ command: 'start' });
                            if (!streamStarted) {
                                // alert('视频流启动失败');
                                return;
                            }
                            // console.log('视频流启动成功');
                        }
                        // this.closeSession();
                        // 显示视频：初始化GstWebRTC
                        this.initGstWebRTCAPI(() => {
                            videoA.setAttribute('visible', true);
                            this.connectingStatus = false;
                        });
                    }
                } catch (error) {
                    console.error('操作失败:', error);
                    // alert(`操作失败：${error.message}`);
                }
            }
        });

        // 页面按钮：切换桌面视频
        const toggleBtn = document.getElementById('toggleVideoBtn');
        toggleBtn.addEventListener('click', () => {

            const desktopVideo = document.querySelector('#desktop-video');

            if (!this.isVideoVisible) {
                if (!this.pwsState) {
                    alert('Jetson包未setup成功');
                    return;
                }
                if (!this.isConnected) {
                    alert('请先连接WebSocket');
                    return;
                }
                if (!this.videoStreamStatus) {
                    alert('请先启动视频流');
                    return;
                }

                // this.closeSession();
                toggleBtn.disabled = true;
                this.initGstWebRTCAPI(() => {
                    desktopVideo.style.display = 'block';
                    toggleBtn.textContent = '隐藏视频';
                    this.isVideoVisible = true;
                    toggleBtn.disabled = false;
                });

            } else {
                desktopVideo.srcObject = null;
                desktopVideo.style.display = 'none';
                toggleBtn.textContent = '显示视频';
                this.closeSession();
                this.isVideoVisible = false;
                toggleBtn.disabled = false;
            }


        });
        toggleBtn.style.visibility = 'visible';

        // 连接/断开WebSocket按钮
        this.connectVideoBtn.addEventListener('click', async () => {
            if (this.isConnected) {
                if (this.websocket) {
                    this.websocket.close();
                }
            } else {
                try {
                    if (!this.pwsState) {
                        alert('Jetson包未setup成功');
                        return;
                    }
                    await this.connectWebSocketAsync();
                } catch (error) {
                    alert(`连接失败：${error.message}`);
                }
            }
        });

        // 启动/停止视频流按钮
        this.startVideoBtn.addEventListener('click', async () => {
            if (!this.pwsState) {
                alert('Jetson包未setup成功');
                return;
            }
            if (!this.isConnected) {
                alert('请先连接WebSocket');
                return;
            }

            const command = this.videoStreamStatus ? 'stop' : 'start';
            await this.sendWebSocketData({ command });
        });
    },

    tick: function () {
        // 原有的tick逻辑（如需启用可取消注释）
        // if (this.rightGripDown) {
        //   const isVisible = this.videoA.getAttribute('visible');
        //   if (!isVisible) return;
        //   try {
        //     const currentControllerPos = this.rightHand.object3D.position;
        //     const delta = {
        //       x: currentControllerPos.x - this.startControllerPos.x,
        //       y: currentControllerPos.y - this.startControllerPos.y,
        //       z: currentControllerPos.z - this.startControllerPos.z
        //     };
        //     const newPos = {
        //       x: this.startVideoPos.x + delta.x,
        //       y: this.startVideoPos.y + delta.y,
        //       z: this.startVideoPos.z + delta.z
        //     };
        //     this.videoA.setAttribute('position', newPos);
        //   } catch (e) {
        //     console.error(e);
        //   }
        // }
    }
});