import React from 'react';
import clsx from 'clsx';
import styles from './Ros2Concept.module.css';

const FeatureList = [
  {
    title: 'Nodes',
    description: (
      <>
        Nodes are the fundamental building blocks of ROS 2. Each node performs a specific function
        and communicates with other nodes through topics, services, and actions.
      </>
    ),
  },
  {
    title: 'Topics',
    description: (
      <>
        Topics enable asynchronous communication between nodes using a publish-subscribe pattern.
        Publishers send data to topics and subscribers receive data from topics.
      </>
    ),
  },
  {
    title: 'Services',
    description: (
      <>
        Services provide synchronous request-response communication patterns between nodes.
        A client sends a request and waits for a response from the server.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function Ros2Concept() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}