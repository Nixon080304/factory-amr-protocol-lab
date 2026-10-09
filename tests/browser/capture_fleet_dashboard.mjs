// SPDX-License-Identifier: Apache-2.0
// Capture the production dashboard URL printed by the real simulation driver.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { chrome, waitFor } from './chrome_cdp.mjs';

const [url, output] = process.argv.slice(2);
assert.match(url, /^http:\/\/127\.0\.0\.1:\d+$/);
assert.ok(output);
const browser = await chrome();
try {
  await browser.send('Page.enable');
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1100, deviceScaleFactor: 1, mobile: false });
  await browser.send('Page.navigate', { url });
  await waitFor(() => browser.evaluate("document.querySelector('#connection')?.dataset.state === 'live'"), 'real fleet dashboard live');
  assert.equal(await browser.evaluate("document.querySelectorAll('[data-robot-id]').length"), 2);
  assert.equal(await browser.evaluate('document.documentElement.scrollWidth <= innerWidth'), true);
  const screenshot = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  await writeFile(output, Buffer.from(screenshot.data, 'base64'));
  console.log(`Captured live production fleet dashboard: ${output}`);
} finally {
  await browser.close();
}
