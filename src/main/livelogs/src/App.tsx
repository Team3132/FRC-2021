import { ChevronDownIcon, LinkIcon, MoonIcon, SunIcon } from "@chakra-ui/icons";
import {
  Center,
  Code,
  Flex,
  Heading,
  IconButton,
  Spacer,
  Text,
  useColorMode,
} from "@chakra-ui/react";
import * as CSS from "csstype";
import { uniqueId } from "lodash";
import React, { useEffect, useMemo, useRef, useState } from "react";
import { useHistory, useLocation } from "react-router-dom";
import useWebSocket, { ReadyState } from "react-use-websocket";
import { Virtuoso } from "react-virtuoso";
function App() {
  /**
   * Defines the current color mode for theming purposes. e.g. dark or light mode.
   */
  const { colorMode, toggleColorMode } = useColorMode();

  /**
   * Defines the browser history so that location can be edited.
   */
  const history = useHistory();

  /**
   * The search parameters from the url
   */
  const search = useLocation().search;

  /**
   * The initial parameters to be read as strings and determine if they're active.
   */
  var initParams = new URLSearchParams(search);

  /**
   * The reference so that the scroll location can be changed.
   */
  const virtuosoRef = useRef(null);

  /**
   * Defines if the user has enabled autoscroll.
   */
  var [autoscroll, setAutoscroll] = useState<boolean>(
    initParams.get("autoscroll") === "true" || undefined
  );

  /**
   * The websocket hook for state changes.
   */
  const { lastMessage, readyState } = useWebSocket(
    `ws://${window.location.hostname}:5803`,
    {
      onError: (error) => {
        console.log(error);
      },
      share: true,
    }
  );

  /**
   * A React ref, stores a mutable value (in this case the current log history).
   */
  const messageHistory = useRef([]);
  messageHistory.current = useMemo(() => {
    // In development there are duplicate messages displayed, this is not the case after build
    if (!lastMessage) return [];
    var splitmsg = lastMessage.data.split(/^([0-9.]+) \((.+)\) \[([^]+)] (.*)/);
    var timestamp = splitmsg[1];
    var severity = splitmsg[2];
    var subsystem = splitmsg[3];
    var content = splitmsg[4];
    var id = uniqueId();
    return messageHistory.current.concat({
      timestamp,
      severity,
      subsystem,
      content,
      id,
    });
  }, [lastMessage]);

  /**
   * The function that listens to changes in the scroll variables and if autoscroll is enabled scroll to the last message.
   */
  useEffect(() => {
    if (autoscroll) {
      virtuosoRef.current.scrollToIndex({
        index: messageHistory.current.length - 1,
        behavior: "smooth",
      });
    }
    // eslint-disable-next-line
  }, [messageHistory.current, autoscroll]);

  /**
   * Adds additional parameters to the URL or deletes them if their state changes.
   */
  useEffect(() => {
    const params = new URLSearchParams();
    if (autoscroll) {
      params.append("autoscroll", "true");
    } else {
      params.delete("autoscroll");
    }
    history.replace({
      search: params.toString(),
    });
  }, [autoscroll, history]);

  /**
   * The reusable row (containing a single line from the log) for virtuoso. Virtuoso dynamically fetches the data for every new Row of items as they are made visible. This row defines how said row should look and where the data should be fetched from.
   * @param index The index of the current displayed message
   * @returns UI message component to be displayed.
   */
  const Row = (index) => {
    if (!messageHistory.current[index]) return <>Zero Size</>;
    const { id, timestamp, severity, subsystem, content } =
      messageHistory.current[index];
    var color: CSS.Property.Color;
    var shade = colorMode === "dark" ? "300" : "500";

    switch (severity) {
      case "Info":
        color = "blue";
        break;
      case "Debug":
        color = "green";
        break;
      case "Error":
        color = "red";
        break;
      case "Warning":
        color = "yellow";
    }

    return (
      <Text textColor={`${color}.${shade}`} w="100%" id={id}>
        {id} {timestamp} ({severity}) [{subsystem}] {content}
      </Text>
    );
  };

  /**
   * Main render for the app.
   */
  return (
    <Flex h="100vh" direction="column">
      <Flex>
        <Center pl={4}>
          <Heading size={"lg"}>Robot Logs</Heading>
        </Center>
        <Spacer />
        <IconButton
          aria-label="autoscroll"
          icon={<ChevronDownIcon />}
          onClick={() => setAutoscroll((prevState) => !prevState)}
          colorScheme={autoscroll ? "blue" : undefined}
          m={2}
        />
        <IconButton
          aria-label="color mode"
          icon={colorMode === "dark" ? <MoonIcon /> : <SunIcon />}
          onClick={toggleColorMode}
          m={2}
        />
        <IconButton
          aria-label="connect"
          icon={<LinkIcon />}
          m={2}
          colorScheme={
            {
              [ReadyState.CONNECTING]: "yellow",
              [ReadyState.OPEN]: "green",
              [ReadyState.CLOSING]: "yellow",
              [ReadyState.CLOSED]: "red",
              [ReadyState.UNINSTANTIATED]: undefined,
            }[readyState]
          }
        />
      </Flex>
      <Code w="100%" flex="1" colorScheme={"blackAlpha"}>
        <Virtuoso
          totalCount={messageHistory.current.length}
          itemContent={Row}
          ref={virtuosoRef}
        />
      </Code>
    </Flex>
  );
}

export default App;
