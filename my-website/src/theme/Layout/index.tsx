import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '../../components/Chatbot';
import type {Props} from '@theme/Layout';

// Define the props for the Layout component
type LayoutProps = Props & {
  children: React.ReactNode;
};

export default function Layout(props: LayoutProps): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <Chatbot />
    </>
  );
}